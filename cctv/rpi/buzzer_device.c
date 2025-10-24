#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/pwm.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define DEVICE_NAME "buzzer"
#define DRIVER_NAME "buzzer"

static struct pwm_device *pwm_buzzer;
static struct timer_list buzzer_timer;

/* ---------------- 타이머 콜백 ---------------- */
static void buzzer_timer_callback(struct timer_list *t)
{
    pwm_disable(pwm_buzzer);
    pr_info("buzzer: stopped after 10 seconds\n");
}

/* ---------------- 파일 오퍼레이션 ---------------- */
static int buzzer_open(struct inode *inode, struct file *file)
{
    pr_info("buzzer: device opened\n");
    return 0;
}

static ssize_t buzzer_write(struct file *file, const char __user *buf,
                            size_t len, loff_t *off)
{
    char kbuf[32];

    if (len >= sizeof(kbuf))
        return -EINVAL;
    if (copy_from_user(kbuf, buf, len))
        return -EFAULT;
    kbuf[len] = '\0';

    /* 입력 문자열 앞뒤 공백/개행 제거 */
    strim(kbuf);

    printk(KERN_INFO "buzzer: received '%s'\n", kbuf);

    /* OFF */
    if (!strcmp(kbuf, "0")) {
        pwm_disable(pwm_buzzer);
        del_timer(&buzzer_timer);
        printk(KERN_INFO "buzzer: stopped\n");
        return len;
    }

    /* 기본: 2 kHz, 50% duty */
    if (!strcmp(kbuf, "1")) {
        unsigned int freq = 2000;
        unsigned int duty = 50;
        unsigned int period_ns = 1000000000U / freq;
        unsigned int duty_ns   = (period_ns * duty) / 100;

        pwm_config(pwm_buzzer, duty_ns, period_ns);
        pwm_enable(pwm_buzzer);
        mod_timer(&buzzer_timer, jiffies + msecs_to_jiffies(10000));
        printk(KERN_INFO "buzzer: started %u Hz, %u%% duty for 10s\n", freq, duty);
        return len;
    }

    /* 사용자 지정 "freq duty" 입력 처리 */
    {
        unsigned int freq = 0, duty = 0;
        if (sscanf(kbuf, "%u %u", &freq, &duty) == 2) {
            if (freq < 50) freq = 50;           /* 최소 50 Hz */
            if (freq > 10000) freq = 10000;     /* 최대 10 kHz */
            if (duty > 100) duty = 50;          /* duty 0~100% */

            unsigned int period_ns = 1000000000U / freq;
            unsigned int duty_ns   = (period_ns * duty) / 100;

            pwm_config(pwm_buzzer, duty_ns, period_ns);
            pwm_enable(pwm_buzzer);
            mod_timer(&buzzer_timer, jiffies + msecs_to_jiffies(10000));
            printk(KERN_INFO "buzzer: started custom %u Hz, %u%% duty\n", freq, duty);
        } else {
            printk(KERN_WARNING "buzzer: invalid input '%s'\n", kbuf);
        }
    }

    return len;
}
static int buzzer_release(struct inode *inode, struct file *file)
{
    pr_info("buzzer: device closed\n");
    return 0;
}

static struct file_operations buzzer_fops = {
    .owner   = THIS_MODULE,
    .open    = buzzer_open,
    .write   = buzzer_write,
    .release = buzzer_release,
};

/* ---------------- misc device ---------------- */
static struct miscdevice buzzer_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = DEVICE_NAME,
    .fops  = &buzzer_fops,
};

/* ---------------- OF 매칭 테이블 ---------------- */
static const struct of_device_id buzzer_of_match[] = {
    { .compatible = "gpio-buzzer" },
    { }
};
MODULE_DEVICE_TABLE(of, buzzer_of_match);

/* ---------------- probe / remove ---------------- */
static int buzzer_probe(struct platform_device *pdev)
{
    int ret;

    pr_info("buzzer: probe called\n");

    /* DTS의 pwms 속성 가져오기 */
    pwm_buzzer = devm_pwm_get(&pdev->dev, NULL);
    if (IS_ERR(pwm_buzzer)) {
        dev_err(&pdev->dev, "failed to get PWM\n");
        return PTR_ERR(pwm_buzzer);
    }

    /* timer 초기화 */
    timer_setup(&buzzer_timer, buzzer_timer_callback, 0);

    /* /dev/buzzer 등록 */
    ret = misc_register(&buzzer_misc_device);
    if (ret) {
        dev_err(&pdev->dev, "failed to register misc device\n");
        return ret;
    }

    pr_info("buzzer: driver initialized successfully\n");
    return 0;
}

static void  buzzer_remove(struct platform_device *pdev)
{
    misc_deregister(&buzzer_misc_device);
    pwm_disable(pwm_buzzer);
    pr_info("buzzer: removed\n");
}

static struct platform_driver buzzer_driver = {
    .probe  = buzzer_probe,
    .remove = buzzer_remove,
    .driver = {
        .name           = DRIVER_NAME,
        .of_match_table = buzzer_of_match,
    },
};

module_platform_driver(buzzer_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("PWM Buzzer Driver with Device Tree support");

