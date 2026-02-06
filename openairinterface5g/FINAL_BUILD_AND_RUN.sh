#!/bin/bash
# Project_CSIplusMatlab_13bit - 최종 빌드 및 실행 스크립트
# 원본(Project_CSIplusMatlab)과 동일하게 USRP 라이브러리를 포함하여 빌드

set -e

# 스크립트 위치를 기준으로 프로젝트 루트 자동 감지
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
BUILD_DIR="$PROJECT_ROOT/cmake_targets/ran_build/build"

cd "$PROJECT_ROOT"

echo "=========================================="
echo "USRP 라이브러리 포함 빌드 시작"
echo "=========================================="
echo ""

# Step 1: 빌드 디렉토리 정리
echo "[1/4] 빌드 디렉토리 정리 중..."
rm -rf cmake_targets/ran_build/build
echo "✓ 완료"

# Step 2: USRP 활성화하여 빌드
echo ""
echo "[2/4] USRP 활성화하여 빌드 중..."
echo "빌드 옵션: --nrUE --gNB -w SIMU -w USRP"
./cmake_targets/build_oai --nrUE --gNB -w SIMU -w USRP

# Step 3: 빌드 결과 확인
echo ""
echo "[3/4] 빌드 결과 확인 중..."
MISSING=0

if [ ! -f "$BUILD_DIR/liboai_device.so" ]; then
    echo "✗ liboai_device.so 없음"
    MISSING=1
fi

if [ ! -f "$BUILD_DIR/liboai_usrpdevif.so" ]; then
    echo "✗ liboai_usrpdevif.so 없음"
    MISSING=1
fi

if [ ! -f "$BUILD_DIR/liboai_iqplayer.so" ]; then
    echo "✗ liboai_iqplayer.so 없음"
    MISSING=1
fi

if [ $MISSING -eq 0 ]; then
    echo "✓ 모든 필수 라이브러리 파일 확인:"
    ls -lh "$BUILD_DIR"/liboai*.so
    echo ""
    echo "✓ CMakeCache 확인:"
    grep "OAI_USRP:BOOL" "$BUILD_DIR/CMakeCache.txt"
    echo ""
    echo "✓ RUNPATH 확인:"
    readelf -d "$BUILD_DIR/nr-uesoftmodem" | grep RUNPATH
else
    echo "✗ 빌드 실패: 일부 라이브러리 파일이 생성되지 않았습니다"
    exit 1
fi

# Step 4: 실행 명령어 출력
echo ""
echo "[4/4] 빌드 완료!"
echo ""
echo "=========================================="
echo "실행 명령어 (추가 환경변수 불필요):"
echo "=========================================="
echo ""
echo "cd $BUILD_DIR"
echo "sudo ./nr-uesoftmodem --usrp-args \"addr=192.168.20.2,clock_source=internal,time_source=internal\" \\"
echo "  -O ../../../targets/PROJECTS/GENERIC-NR-5GC/CONF/nrue.uicc.conf \\"
echo "  -C 3609120000 -r 51 --numerology 1 --ssb 234 --ue-nb-ant-rx 4 --ue-nb-ant-tx 1"
echo ""
