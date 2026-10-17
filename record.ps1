# Ubuntuデスクトップ(コンテナ内の画面 :0)だけを録画し、video\ に保存する
# Windows側の画面は映らない。
# 使い方: .\record.ps1 -Seconds 60 -Name deploy_pekori
#   -Seconds : 録画する秒数
#   -Name    : 保存ファイル名(拡張子なし)。video\<Name>.mp4 になる
param(
    [int]$Seconds = 30,
    [string]$Name = ("rec_" + (Get-Date -Format "yyyyMMdd_HHmmss"))
)

$Tmp = "/tmp/$Name.mp4"
$Out = Join-Path $PSScriptRoot "video\$Name.mp4"

Write-Host "録画開始: $Seconds 秒 → $Out"
docker exec -u ubuntu pai_ros2 ffmpeg -y -loglevel error `
    -f x11grab -framerate 15 -i :0 -t $Seconds `
    -c:v libx264 -preset ultrafast -pix_fmt yuv420p $Tmp
docker cp "pai_ros2:$Tmp" $Out
docker exec pai_ros2 rm -f $Tmp
Write-Host "保存しました: $Out"
