#!/bin/bash
# Project_CSIplusMatlab_13bit용 USRP 라이브러리 빌드 스크립트
# 원본(Project_CSIplusMatlab)과 동일하게 liboai_device.so를 생성하기 위한 빌드 절차

set -e

# 스크립트 위치를 기준으로 프로젝트 루트 자동 감지
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
cd "$PROJECT_ROOT"

echo "=========================================="
echo "Step 1: 기존 빌드 디렉토리 정리"
echo "=========================================="
rm -rf cmake_targets/ran_build/build
echo "✓ 빌드 디렉토리 삭제 완료"

echo ""
echo "=========================================="
echo "Step 2: USRP 활성화하여 빌드 시작"
echo "=========================================="
echo "빌드 옵션: --nrUE --gNB -w SIMU -w USRP"
echo ""

# USRP와 SIMU를 모두 활성화하여 빌드
./cmake_targets/build_oai --nrUE --gNB -w SIMU -w USRP 2>&1 | tee /tmp/oai_build_usrp.log

echo ""
echo "=========================================="
echo "Step 3: 빌드 결과 확인"
echo "=========================================="

# 필수 라이브러리 파일 확인
BUILD_DIR="$PROJECT_ROOT/cmake_targets/ran_build/build"
MISSING_FILES=()

if [ ! -f "$BUILD_DIR/liboai_device.so" ]; then
    MISSING_FILES+=("liboai_device.so")
fi

if [ ! -f "$BUILD_DIR/liboai_usrpdevif.so" ]; then
    MISSING_FILES+=("liboai_usrpdevif.so")
fi

if [ ! -f "$BUILD_DIR/liboai_iqplayer.so" ]; then
    MISSING_FILES+=("liboai_iqplayer.so")
    echo "⚠ liboai_iqplayer.so가 없습니다. 수동 빌드 시도..."
    cd "$BUILD_DIR"
    make oai_iqplayer 2>&1 | tail -5
    cd "$PROJECT_ROOT"
fi

# 재확인
if [ ! -f "$BUILD_DIR/liboai_iqplayer.so" ]; then
    MISSING_FILES+=("liboai_iqplayer.so")
fi

if [ ${#MISSING_FILES[@]} -eq 0 ]; then
    echo "✓ 모든 필수 라이브러리 파일이 생성되었습니다:"
    ls -lh "$BUILD_DIR"/liboai*.so
    echo ""
    echo "✓ 빌드 성공!"
else
    echo "✗ 다음 파일들이 생성되지 않았습니다:"
    for file in "${MISSING_FILES[@]}"; do
        echo "  - $file"
    done
    echo ""
    echo "빌드 로그 확인: /tmp/oai_build_usrp.log"
    exit 1
fi

echo ""
echo "=========================================="
echo "Step 4: CMakeCache 확인"
echo "=========================================="
echo "OAI_USRP 설정:"
grep "OAI_USRP:BOOL" "$BUILD_DIR/CMakeCache.txt" || echo "  (찾을 수 없음)"

echo ""
echo "=========================================="
echo "Step 5: RUNPATH 확인"
echo "=========================================="
echo "nr-uesoftmodem의 RUNPATH:"
readelf -d "$BUILD_DIR/nr-uesoftmodem" | grep -E "RPATH|RUNPATH" || echo "  (RUNPATH 없음)"

echo ""
echo "=========================================="
echo "빌드 완료!"
echo "=========================================="
echo ""
echo "실행 명령어:"
echo "cd $BUILD_DIR"
echo "sudo ./nr-uesoftmodem --usrp-args \"addr=192.168.20.2,clock_source=internal,time_source=internal\" \\"
echo "  -O ../../../targets/PROJECTS/GENERIC-NR-5GC/CONF/nrue.uicc.conf \\"
echo "  -C 3609120000 -r 51 --numerology 1 --ssb 234 --ue-nb-ant-rx 4 --ue-nb-ant-tx 1"
