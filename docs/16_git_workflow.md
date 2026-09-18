# Git 同步流程（Windows ↔ 虚拟机 ↔ 小车）

> 代码在 Windows，看代码/改代码在虚拟机，跑代码在小车。
> 三者之间怎么同步，这份文档说清楚。

---

## 0. 三者分工

```
Windows                     虚拟机 (VS Code)            小车 (Jetson)
C:\...\r2_mapping           ~/r2_mapping                ~/r2_mapping
   |                            |                            |
   |      git push/pull         |      git push/pull         |
   +-------> git 中转站 <-------+                            |
             (GitHub 或裸仓库)                                |
                                |                            |
                                +--------- scp ------------->+

小车不参与 git（它只跑，不改），同步用 scp。
```

---

## 1. 方案 A：用 GitHub 中转（推荐，和你另一个项目一致）

### 1.1 先在网页建空仓库

登录 GitHub，New repository，名字 `r2_mapping`。

**不要勾** Add a README / Add .gitignore / Choose a license —— 要一个空仓库。

### 1.2 Windows 推上去

```powershell
cd C:\Users\zhuchenju\Desktop\建图\r2_mapping
git remote add origin https://github.com/flaashman-009/r2_mapping.git
git push -u origin main
```

第一次会让你输账号或令牌。如果配过 SSH key，也可以用 SSH：

```powershell
git remote set-url origin git@github.com:flaashman-009/r2_mapping.git
```

### 1.3 虚拟机上拉下来

```bash
cd ~
git clone https://github.com/flaashman-009/r2_mapping.git
cd r2_mapping
code .                    # 用 VS Code 打开
```

以后在虚拟机上改完代码：

```bash
cd ~/r2_mapping
git add -A
git commit -m "说明改了什么"
git push origin main
```

Windows 那边拉取：

```powershell
cd C:\Users\zhuchenju\Desktop\建图\r2_mapping
git pull
```

---

## 2. 方案 B：不用 GitHub，直接推给虚拟机

前提：虚拟机开了 SSH 服务，Windows 能 ssh 到它。

### 2.1 虚拟机上建裸仓库

```bash
mkdir -p ~/repos/r2_mapping.git
cd ~/repos/r2_mapping.git
git init --bare
```

### 2.2 Windows 推上去

```powershell
cd C:\Users\zhuchenju\Desktop\建图\r2_mapping
git remote add origin ssh://flaash@<虚拟机IP>/home/flaash/repos/r2_mapping.git
git push -u origin main
```

### 2.3 虚拟机上 clone 出来

```bash
cd ~
git clone ~/repos/r2_mapping.git r2_mapping
cd r2_mapping
code .
```

---

## 3. 改完之后怎么弄到车上（必须做）

车上的代码不会自己更新。

```powershell
scp -r .\config .\scripts .\src .\docs .\tools jetson@<车IP>:~/r2_mapping/
```

或者只传改动的那个：

```powershell
scp .\config\nav_r2.yaml jetson@<车IP>:~/r2_mapping/config/
scp .\scripts\nav.sh jetson@<车IP>:~/r2_mapping/scripts/
```

传完在车上验证语法：

```bash
bash -n ~/r2_mapping/scripts/nav.sh
python3 -m py_compile ~/r2_mapping/scripts/nav_watch.py
bash ~/r2_mapping/scripts/verify_changes.sh
```

---

## 4. 仓库里有什么 / 没什么

| 进了 git | 没进 git | 原因 |
|---|---|---|
| 所有代码、配置、文档 | `maps/*.pgm`、`maps/*.png` | 地图数据，体积大 |
| `maps/*.yaml` | `logs/*.csv`、`logs/*.png` | 每次跑车都产生 |
| `logs/README.md` | `bags/*` | 录包数据 |
| | `build/ install/ log/`、`__pycache__/` | 编译产物 |

想强行把某次记录也存进去：

```bash
git add -f logs/nav_watch_0918_100241.csv
```

---

## 5. 行尾问题（重要，别跳过）

仓库里有 `.gitattributes`，强制所有文本文件用 **LF**。

**为什么必须有**：Windows 的 git 默认把 LF 转成 CRLF。shell 脚本一旦变成 CRLF，
拿到 Linux 上会报：

```
bad interpreter: /bin/bash^M: no such file or directory
```

一行都跑不了。有 `.gitattributes` 就不会发生。

**如果已经中招**（虚拟机里 `bash xxx.sh` 报上面的错），这样救：

```bash
cd ~/r2_mapping
git config core.autocrlf false
git rm --cached -r . >/dev/null
git reset --hard
```
