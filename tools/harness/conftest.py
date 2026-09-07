import os
import sys

# 패키지 루트(setup.py 가 있는 디렉터리)를 import 경로에 추가 → colcon 설치 없이 소스 트리에서 테스트
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
