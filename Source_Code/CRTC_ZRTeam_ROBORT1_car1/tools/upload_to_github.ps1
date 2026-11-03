<#
  把本工程（CRTC_ZRTeam_ROBORT1）当前提交的快照上传到 GitHub 团队的 Source_Code/ 目录。

  远端布局：
    Source_Code/CRTC_ZRTeam_ROBORT1/   工程本体（来自本仓库 HEAD）
    Source_Code/docs/                  本仓库 docs/ 下的文档
    （团队原有的 README.md、Source_Code/log、docs/README.md 都不动）

  只上传 git 已跟踪的文件（不含 .idea、cmake-build-* 等构建产物）。
  每次运行产生一个提交；本仓库自己的提交历史与目录结构不受影响。

  用法（在仓库根目录执行，Windows PowerShell 5.1 或 PowerShell 7 均可）：
    powershell -NoProfile -File tools/upload_to_github.ps1
    powershell -NoProfile -File tools/upload_to_github.ps1 -Message "update remote mapping"
  已装 PowerShell 7 时把 powershell 换成 pwsh 亦可。

  说明：本文件带 UTF-8 BOM —— Windows PowerShell 5.1 对无 BOM 的脚本会按 ANSI 解码，
        中文注释会变乱码并导致语法错误。改动本文件后请确认仍保留 BOM。
#>
param(
    [string]$Message   = "",
    [string]$RemoteUrl = 'https://github.com/SakuraAz0/The_log_of_Zero_Reapers_Team',
    [string]$Branch    = 'main'
)

# 不使用 $ErrorActionPreference='Stop'：git/tar 会把正常信息写到 stderr，
# 在 PS 5.1 下会变成错误记录并中止脚本。改为每步显式检查 $LASTEXITCODE。
$ErrorActionPreference = 'Continue'
$env:GIT_TERMINAL_PROMPT = '0'      # 非交互环境下不要让 git 卡住等待输入

$root = Split-Path -Parent $PSScriptRoot          # 仓库根目录
$tmp  = Join-Path $root '_gh_upload'              # 临时克隆（已忽略，不会提交）
$tar  = Join-Path $root '_gh_src.tar'

if (-not $Message) {
    $head = (git -C $root log -1 --pretty=%s)
    $Message = "Update CRTC_ZRTeam_ROBORT1 source code ($head)"
}

Write-Host "[1/5] 导出已跟踪文件：git archive HEAD（排除 docs，中文名文档另行复制）"
if (Test-Path $tar) { Remove-Item $tar -Force }
git -C $root archive --format=tar -o $tar HEAD -- . ':(exclude)docs'
if ($LASTEXITCODE -ne 0) { throw "git archive 失败" }

Write-Host "[2/5] 准备远端克隆：$tmp"
if (Test-Path (Join-Path $tmp '.git')) {
    git -C $tmp fetch -q origin $Branch
    git -C $tmp checkout -q $Branch 2>$null
    git -C $tmp reset -q --hard "origin/$Branch"
    git -C $tmp clean -qfd
} else {
    if (Test-Path $tmp) { Remove-Item $tmp -Recurse -Force }
    git clone -q $RemoteUrl $tmp
    git -C $tmp checkout -q $Branch 2>$null
}
if ($LASTEXITCODE -ne 0) { throw "克隆/更新远端失败（检查网络与凭据）" }

Write-Host "[3/5] 替换 Source_Code/CRTC_ZRTeam_ROBORT1 与 Source_Code/docs"
$dstProj = Join-Path $tmp 'Source_Code\CRTC_ZRTeam_ROBORT1'
$dstDocs = Join-Path $tmp 'Source_Code\docs'
foreach ($d in @($dstProj, $dstDocs)) {
    if (Test-Path $d) { Remove-Item $d -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $d | Out-Null
}

tar -xf $tar -C $dstProj 2>$null
if ($LASTEXITCODE -ne 0) { throw "解压归档失败" }

# 文档用 PowerShell 原生复制（bsdtar 在 Windows 上处理中文文件名会报 Invalid empty pathname）
$srcDocs = Join-Path $root 'docs'
if (Test-Path $srcDocs) {
    Get-ChildItem -LiteralPath $srcDocs -File | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $dstDocs -Force
    }
}

# 数量核对：不允许静默丢文件
$want = (git -C $root ls-files | Measure-Object).Count
$got  = (Get-ChildItem $dstProj -Recurse -File -Force | Measure-Object).Count +
        (Get-ChildItem $dstDocs -Recurse -File -Force | Measure-Object).Count
Write-Host ("      本仓库跟踪 {0} 个文件，已放入远端 {1} 个" -f $want, $got)
if ($got -ne $want) { throw "文件数量不一致（$got/$want），已中止，未推送" }

Write-Host "[4/5] 提交"
git -C $tmp add -A
$changes = (git -C $tmp status --short | Measure-Object).Count
if ($changes -eq 0) {
    Write-Host "      远端已是最新，无需提交"
} else {
    git -C $tmp commit -q -m $Message
    if ($LASTEXITCODE -ne 0) { throw "提交失败" }
    Write-Host "[5/5] 推送到 origin/$Branch"
    git -C $tmp push origin $Branch 2>&1 | Select-Object -Last 3
    if ($LASTEXITCODE -ne 0) { throw "推送失败（检查 GitHub 凭据/权限）" }
    Write-Host ("      完成：{0} 个变更已推送" -f $changes)
}

Remove-Item $tar -Force -ErrorAction SilentlyContinue
Write-Host "远端地址：$RemoteUrl"
