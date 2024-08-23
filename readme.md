### CADC2024横列双发尾座垂直起降飞控  
1. 装软件：  
    1. Keil  
    2. Git  
    3. Visual Studio Code  

2. 配环境
    1. 让TC把你拉进git仓库（private repository）
    2. VSCode登录github账号
    3. Clone飞控仓库

3. 日常维护：  
    1. 拉取、选分支，编译与下载
    2. 进入VSCode根据飞行器情况（比赛机or训练机，碳桨or尼龙桨），checkout到对应分支
    3. 进入Keil，编译与下载
    4. 不同配置一般启动音乐不同

4. 开发：
    1. 修改飞控代码前后创建一个新命名的分支
    2. **正式分支禁止直接推送代码**，仅允许合并后提Pull Request
    3. 如果不想合并进几个主分支，就将主分支更新合并进自己的分支  
  