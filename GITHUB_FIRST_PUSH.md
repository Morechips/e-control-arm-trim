# 第一次关联GitHub并推送：当前v4.5

**用户已恢复首次上传，先保存当前 v4.5 软件基线。抓人质姿态待重录、实车验证待完成；上传源码不代表完成验收。暂不创建“完整抓取验收”发布。重录后重新构建、更新固件哈希和版本说明。**

已准备目录：`D:\工科大\e-control-arm-trim-github`。此目录已初始化 Git，分支 main；执行首次上传前先用 status/log/remote 确认进度，不要重复 init 或 remote add。源码与测试副本对应，上传忽略规则已准备。当前可运行测试目录与原团队Git仓库保留。

2026-10-02 所有权问题已定位：工作目录属于 CodexSandboxOffline，而用户是 ljn。已在 `C:\Users\ljn\.gitconfig` 为 `D:/工科大/e-control-arm-trim-github` 添加单目录 safe.directory。用户如在另一环境仍报同样错误，先检查 `git config --global --show-origin --get-all safe.directory`，不要只复制错误提示而跳过执行。

## 1. 你在浏览器创建空仓库

登录GitHub，打开 https://github.com/new 。Owner选择自己的账号，Repository name建议e-control-arm-trim；开发中可先选Private。不要勾选生成README、.gitignore或License，已有文件在本地。点击Create repository，复制HTTPS仓库地址，例如 `https://github.com/你的用户名/e-control-arm-trim.git`。

参考：[GitHub首次上传官方说明](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github?platform=windows)。

## 2. 在PowerShell建立本地版本

```powershell
Set-Location "D:\工科大\e-control-arm-trim-github"
git init -b main
git config user.name
git config user.email
```

这台电脑已配置作者信息；输出符合你希望使用的身份即可继续。name/email只是提交署名，不是GitHub登录。若要仅对这个仓库更改署名：

```powershell
git config user.name "你希望显示的作者名"
git config user.email "GitHub邮箱或GitHub提供的noreply邮箱"
```

接着：

```powershell
git add .
git status --short
git diff --cached --stat
git commit -m "arm: save v4.5 planar-only reference actions"
```

git init开启本地版本管理，main是主分支。git add把当前修改放进待提交列表，status查看清单；commit把清单保存为本地版本。到这一步仍未上传。

## 3. 关联远程并首次上传

```powershell
$repoUrl = Read-Host "粘贴刚创建的GitHub仓库HTTPS地址"
git remote add origin $repoUrl
git remote -v
git push -u origin main
```

origin是远程仓库的本地名字，remote -v检查地址；push把本地提交上传，-u记住main与origin/main的对应关系，以后可直接git push。电脑已配置Git Credential Manager；首次需要身份认证时通常会打开浏览器，登录拥有该仓库权限的账号并完成授权。登录/验证码由你本人操作，见 [GitHub凭据官方说明](https://docs.github.com/en/get-started/git-basics/caching-your-github-credentials-in-git)。

成功后刷新网页，确认Core、scripts、tests和README出现。如果某一步报错，先处理该步，别继续叠加后续命令。origin已存在时先git remote -v；不要重复add。新建时若勾选了在线README导致远程已有提交，先保留日志并确认实际历史，不用强推覆盖。

## 4. 实车验证完成后打标记并保存镜像

新姿态录好、软件重新检查且实车验证完成，并首次推送 main 成功后：

```powershell
git tag -a v4.5 -m "Planar-only reference actions; see acceptance record"
git push origin v4.5
```

tag固定指向该版本，后续修改不会改变它。GitHub仓库页面进入Releases → Draft a new release，选择已推送的v4.5标签，标题填写v4.5。说明复制RELEASE_NOTES_V4_5.md；附件上传 `D:\工科大\e-control-arm-trim-v4.5-firmware.zip`，再发布。源码按Git提交保存，预编译固件作为Release附件保存，参考 [GitHub Release官方说明](https://docs.github.com/en/repositories/releasing-projects-on-github/managing-releases-in-a-repository)。

## 5. 以后怎么更新

今后的开发应在这个Git目录进行；如果先在别处测试，再把确认过的源码变更同步过来后提交。优化平滑性时：

```powershell
git switch -c improve-arm-smoothing
```

改完、测试通过后：

```powershell
git status
git add .
git diff --cached --stat
git commit -m "arm: improve continuous trim smoothness"
git push -u origin improve-arm-smoothing
```

后续同一分支一般用git push即可；验证好再把该分支合入main。git diff显示尚未暂存的修改，git diff --cached显示将进入提交的修改。源码不会因保存到硬盘而自动上传，也不会因commit自动上传；上传动作是push。

## 分工

已经准备：源码目录、忽略规则、当前版本说明、定性实车记录、Release固件包。你最适合负责：创建自己的仓库和选择可见性、浏览器账号登录/双重验证、第一次亲自执行init/add/commit/remote/push熟悉流程。仓库链接提供后，代码整理、差异检查、版本记录和故障诊断可以继续在本任务完成。
