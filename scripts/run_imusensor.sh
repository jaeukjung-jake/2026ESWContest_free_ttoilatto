#!/bin/bash
#
# imusensor.py를 venv 가상환경에서 실행시켜주는 셔틀 스크립트.
# roslaunch가 이 스크립트를 호출합니다.

# 1. imusensor.py를 위한 Python 3.8 가상환경(venv) 활성화
#    (사용자 환경에 맞게 절대 경로를 수정해야 합니다)
source "$HOME/myvenv/bin/activate"

# 2. imusensor.py 스크립트의 절대 경로 찾기
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# 3. 가상환경의 python으로 imusensor.py 실행
exec python3.8 "$SCRIPT_DIR/sensors/imusensor.py"