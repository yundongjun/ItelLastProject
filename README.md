# ItelLastProject

# 처음 시작

1. git 다운로드
2. git init으로 git repository 생성
```
git init
```
3.  원격 repository 가져오기
```
git clone https://github.com/yundongjun/ItelLastProject.git
```
# 폴더형식

1. stm32폴더에는 stm32관련된 내용 (성준)
2. cctv폴더에는 cctv관련된 내용 (박진수)
3. ros폴더에는 ros, opencr 관려된 내용 (윤동준, 설영현)

# git에 올릴때마다 해야될 일

1. 원격 repository 동기화

   1-1. 전에 받았던 내용 기준으로 올릴 경우
   ```
   git pull
   ```
   1-2. 처음부터 다시 시작할 경우
   ```
   git init
   git clone https://github.com/yundongjun/ItelLastProject.git
   ```
3. 자기 자신 repository에 올리기
   ```
   git add .
   ```
4. 자기 자신 repository에 저장
   ```
   git commit -m "<메세지>"
   ```
5. 자기 자신 repository에 있는 내용 원격 repository에 올리기
   ```
   git push
   ```
