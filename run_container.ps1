# コンテナ起動スクリプト(初回のみ。2回目以降は `docker start pai_ros2`)
#  -d          : バックグラウンドで起動(付けないとターミナルが固まる)
#  -p 6080:80  : ブラウザ http://localhost:6080/ でUbuntuデスクトップに接続
#  -p 9090/9876: rerun 可視化用
#  -v          : Windows側 work\ をコンテナ内 /home/ubuntu/work と共有
#  --shm-size  : GUIアプリ(Gazebo等)の共有メモリ不足を防ぐ

$Image   = "airobotbook/ros2-desktop-ai-robot-book-humble"
$WorkDir = Join-Path $PSScriptRoot "work"

docker run -d `
    --name pai_ros2 `
    -p 6080:80 `
    -p 9090:9090 `
    -p 9876:9876 `
    --shm-size=512m `
    -v "${WorkDir}:/home/ubuntu/work" `
    $Image
