#!/bin/bash
# Project_CSIplusMatlab_13bit - liboai_device.so 생성 및 설치를 위한 완전한 빌드 스크립트
# 이 스크립트를 그대로 실행하면 성공적으로 빌드됩니다.

set -e

# 스크립트 위치를 기준으로 프로젝트 루트 자동 감지
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
BUILD_DIR="$PROJECT_ROOT/cmake_targets/ran_build/build"
BUILD_LOG="$PROJECT_ROOT/build_log_liboai_device.txt"

cd "$PROJECT_ROOT"

echo "=========================================="
echo "liboai_device.so 빌드 및 설치 스크립트"
echo "=========================================="
echo "프로젝트 경로: $PROJECT_ROOT"
echo "빌드 디렉토리: $BUILD_DIR"
echo "빌드 로그: $BUILD_LOG"
echo ""

# Step 1: 기존 빌드 디렉토리 완전 정리 (CMakeCache 잔재 제거)
echo "=========================================="
echo "[Step 1/6] 빌드 디렉토리 완전 정리"
echo "=========================================="
if [ -d "$BUILD_DIR" ]; then
    echo "기존 빌드 디렉토리 삭제 중..."
    rm -rf "$BUILD_DIR"
    echo "✓ 빌드 디렉토리 삭제 완료"
else
    echo "✓ 빌드 디렉토리가 없습니다 (정상)"
fi
echo ""

# Step 2: CMakeCache 잔재 확인 및 제거
echo "=========================================="
echo "[Step 2/6] CMakeCache 잔재 확인 및 제거"
echo "=========================================="
# 빌드 디렉토리 내부의 CMakeCache 확인
if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "⚠ 경고: CMakeCache.txt가 여전히 존재합니다."
    echo "  이전 빌드 설정이 남아있어 플러그인 빌드가 스킵될 수 있습니다."
    echo "  CMakeCache 내용 확인:"
    grep -E "OAI_USRP|OAI_SIMU|CMAKE_BUILD_TYPE|CMAKE_CACHEFILE_DIR" "$BUILD_DIR/CMakeCache.txt" | head -5 || true
    echo "  강제 삭제합니다."
    rm -f "$BUILD_DIR/CMakeCache.txt"
fi

# 상위 디렉토리의 CMakeCache도 확인
PARENT_CMAKE_CACHE="$PROJECT_ROOT/cmake_targets/ran_build/CMakeCache.txt"
if [ -f "$PARENT_CMAKE_CACHE" ]; then
    echo "⚠ 경고: 상위 디렉토리에 CMakeCache.txt가 있습니다."
    echo "  $PARENT_CMAKE_CACHE"
    echo "  삭제합니다."
    rm -f "$PARENT_CMAKE_CACHE"
fi

echo "✓ CMakeCache 정리 완료"
echo ""

# Step 3: USRP 활성화하여 빌드 (oai_iqplayer 포함)
echo "=========================================="
echo "[Step 3/6] USRP 활성화하여 빌드 시작"
echo "=========================================="
echo "빌드 옵션 분석:"
echo "  --nrUE          : NR UE 소프트모뎀 빌드"
echo "  --gNB            : gNB 소프트모뎀 빌드"
echo "  -w SIMU          : RF 시뮬레이터 활성화 (항상 빌드됨)"
echo "  -w USRP          : USRP 드라이버 활성화 (liboai_usrpdevif.so 생성)"
echo "  --build-lib oai_iqplayer : IQ 플레이어 라이브러리 빌드"
echo ""
echo "중요: -I 옵션은 사용하지 않습니다 (의존성 설치용, 빌드에는 불필요)"
echo "중요: ENABLE_USRP/ENABLE_B210/UHD는 -w USRP 옵션으로 자동 설정됩니다"
echo ""
echo "빌드 로그는 $BUILD_LOG 에 저장됩니다."
echo ""

# USRP와 SIMU를 모두 활성화하고 oai_iqplayer도 빌드
echo "빌드 명령어 실행 중..."
./cmake_targets/build_oai --nrUE --gNB -w SIMU -w USRP --build-lib oai_iqplayer 2>&1 | tee "$BUILD_LOG"

BUILD_EXIT_CODE=${PIPESTATUS[0]}
if [ $BUILD_EXIT_CODE -ne 0 ]; then
    echo ""
    echo "✗ 빌드 실패 (종료 코드: $BUILD_EXIT_CODE)"
    echo "빌드 로그를 확인하세요: $BUILD_LOG"
    exit 1
fi
echo ""

# Step 4: 빌드 결과 확인
echo "=========================================="
echo "[Step 4/6] 빌드 결과 확인"
echo "=========================================="

MISSING_FILES=()

# 필수 라이브러리 파일 확인
if [ ! -f "$BUILD_DIR/liboai_device.so" ]; then
    MISSING_FILES+=("liboai_device.so")
    echo "✗ liboai_device.so 없음"
else
    echo "✓ liboai_device.so 존재"
    ls -lh "$BUILD_DIR/liboai_device.so"
fi

if [ ! -f "$BUILD_DIR/liboai_usrpdevif.so" ]; then
    MISSING_FILES+=("liboai_usrpdevif.so")
    echo "✗ liboai_usrpdevif.so 없음"
else
    echo "✓ liboai_usrpdevif.so 존재"
    ls -lh "$BUILD_DIR/liboai_usrpdevif.so"
fi

if [ ! -f "$BUILD_DIR/liboai_iqplayer.so" ]; then
    MISSING_FILES+=("liboai_iqplayer.so")
    echo "✗ liboai_iqplayer.so 없음"
else
    echo "✓ liboai_iqplayer.so 존재"
    ls -lh "$BUILD_DIR/liboai_iqplayer.so"
fi

if [ ${#MISSING_FILES[@]} -ne 0 ]; then
    echo ""
    echo "✗ 다음 파일들이 생성되지 않았습니다:"
    for file in "${MISSING_FILES[@]}"; do
        echo "  - $file"
    done
    echo ""
    echo "빌드 로그에서 관련 타겟 확인:"
    echo "  grep -i 'liboai_device\\|liboai_usrpdevif\\|liboai_iqplayer\\|oai_usrpdevif\\|oai_iqplayer' $BUILD_LOG | tail -20"
    exit 1
fi

echo ""
echo "✓ 모든 필수 라이브러리 파일 확인 완료"
echo ""

# CMakeCache에서 OAI_USRP 설정 확인
echo "CMakeCache에서 OAI_USRP 설정 확인:"
if grep -q "OAI_USRP:BOOL=ON" "$BUILD_DIR/CMakeCache.txt" 2>/dev/null; then
    echo "✓ OAI_USRP:BOOL=ON 확인됨"
    grep "OAI_USRP:BOOL" "$BUILD_DIR/CMakeCache.txt"
else
    echo "⚠ 경고: OAI_USRP:BOOL=ON이 CMakeCache에 없습니다"
    grep "OAI_USRP:BOOL" "$BUILD_DIR/CMakeCache.txt" || echo "  (OAI_USRP 관련 설정을 찾을 수 없음)"
fi
echo ""

# 빌드 로그에서 liboai_device.so 생성 타겟 확인
echo "빌드 로그에서 liboai_device.so 생성 타겟 확인:"
echo "  (oai_usrpdevif 타겟이 liboai_device.so를 생성합니다)"
echo ""
echo "  [1] oai_usrpdevif 타겟 빌드 로그:"
grep -i "oai_usrpdevif" "$BUILD_LOG" | grep -E "\[.*%\]|Linking|Building|Built" | tail -10 || echo "    (로그에서 관련 정보를 찾을 수 없음)"
echo ""
echo "  [2] liboai_device.so 심볼릭 링크 생성 로그:"
grep -i "liboai_device\|create_symlink" "$BUILD_LOG" | tail -5 || echo "    (로그에서 관련 정보를 찾을 수 없음)"
echo ""
echo "  [3] oai_iqplayer 타겟 빌드 로그:"
grep -i "oai_iqplayer" "$BUILD_LOG" | grep -E "\[.*%\]|Linking|Building|Built" | tail -5 || echo "    (로그에서 관련 정보를 찾을 수 없음)"
echo ""
echo "  [4] 전체 빌드 타겟 목록 (liboai 관련):"
grep -i "liboai" "$BUILD_LOG" | grep -E "\[.*%\]" | tail -10 || echo "    (로그에서 관련 정보를 찾을 수 없음)"
echo ""

# Step 5: 라이브러리 설치 (권장 방법)
echo "=========================================="
echo "[Step 5/6] 라이브러리 설치 (/usr/local/lib)"
echo "=========================================="
echo "liboai_device.so를 시스템 라이브러리 경로에 설치합니다."
echo "이렇게 하면 LD_LIBRARY_PATH 없이도 실행할 수 있습니다."
echo ""

# sudo 권한 확인
if ! sudo -n true 2>/dev/null; then
    echo "sudo 권한이 필요합니다. 비밀번호를 입력하세요:"
    sudo -v
fi

# /usr/local/lib 디렉토리 생성 (없는 경우)
sudo mkdir -p /usr/local/lib

# 라이브러리 파일 복사
echo "라이브러리 파일 복사 중..."
sudo cp -f "$BUILD_DIR/liboai_usrpdevif.so" /usr/local/lib/
sudo cp -f "$BUILD_DIR/liboai_iqplayer.so" /usr/local/lib/

# liboai_device.so 심볼릭 링크 생성
if [ -L /usr/local/lib/liboai_device.so ]; then
    sudo rm -f /usr/local/lib/liboai_device.so
fi
sudo ln -sf /usr/local/lib/liboai_usrpdevif.so /usr/local/lib/liboai_device.so

echo "✓ 라이브러리 파일 설치 완료"
echo ""

# ldconfig 실행
echo "ldconfig 실행 중..."
sudo ldconfig
echo "✓ ldconfig 완료"
echo ""

# 설치 확인
echo "설치된 라이브러리 확인:"
ldconfig -p | grep liboai_device || echo "  (liboai_device.so를 찾을 수 없음)"
ldconfig -p | grep liboai_usrpdevif || echo "  (liboai_usrpdevif.so를 찾을 수 없음)"
ldconfig -p | grep liboai_iqplayer || echo "  (liboai_iqplayer.so를 찾을 수 없음)"
echo ""

# Step 6: 최종 확인
echo "=========================================="
echo "[Step 6/6] 최종 확인"
echo "=========================================="

# 빌드 디렉토리의 파일 확인
echo "빌드 디렉토리의 파일:"
find "$BUILD_DIR" -maxdepth 1 -name "liboai_device.so" -o -name "liboai_usrpdevif.so" -o -name "liboai_iqplayer.so" | sort
echo ""

# 설치된 파일 확인
echo "설치된 파일:"
ls -lh /usr/local/lib/liboai*.so 2>/dev/null || echo "  (설치된 파일이 없습니다)"
echo ""

# nr-uesoftmodem 바이너리 확인
if [ -f "$BUILD_DIR/nr-uesoftmodem" ]; then
    echo "nr-uesoftmodem 바이너리 정보:"
    echo "  파일 크기: $(ls -lh "$BUILD_DIR/nr-uesoftmodem" | awk '{print $5}')"
    echo "  RUNPATH 확인:"
    readelf -d "$BUILD_DIR/nr-uesoftmodem" | grep -E "RUNPATH|RPATH" || echo "    (RUNPATH/RPATH 없음)"
    echo ""
fi

echo "=========================================="
echo "빌드 및 설치 완료!"
echo "=========================================="
echo ""
echo "다음 명령어로 실행할 수 있습니다 (LD_LIBRARY_PATH 불필요):"
echo ""
echo "  cd $BUILD_DIR"
echo "  sudo ./nr-uesoftmodem [옵션들]"
echo ""
echo "또는 빌드 디렉토리에서 직접 실행:"
echo ""
echo "  cd $BUILD_DIR"
echo "  sudo LD_LIBRARY_PATH=. ./nr-uesoftmodem [옵션들]"
echo ""
