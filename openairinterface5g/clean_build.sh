#!/bin/bash
# 클린 빌드 스크립트 - CMake 캐시 및 빌드 산출물 완전 정리
# 프로젝트 폴더 이름이 변경되어도 안전하게 작동합니다.

set -e

# 스크립트 위치를 기준으로 프로젝트 루트 자동 감지
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
BUILD_DIR="$PROJECT_ROOT/cmake_targets/ran_build/build"

cd "$PROJECT_ROOT"

echo "=========================================="
echo "클린 빌드 스크립트"
echo "=========================================="
echo "프로젝트 경로: $PROJECT_ROOT"
echo "빌드 디렉토리: $BUILD_DIR"
echo ""

# Step 1: 빌드 디렉토리 완전 삭제
echo "[1/4] 빌드 디렉토리 삭제 중..."
if [ -d "$BUILD_DIR" ]; then
    rm -rf "$BUILD_DIR"
    echo "✓ 빌드 디렉토리 삭제 완료"
else
    echo "✓ 빌드 디렉토리가 없습니다 (정상)"
fi
echo ""

# Step 2: CMake 캐시 파일 정리
echo "[2/4] CMake 캐시 파일 정리 중..."
CMAKE_CACHE_FILES=(
    "$BUILD_DIR/CMakeCache.txt"
    "$PROJECT_ROOT/cmake_targets/ran_build/CMakeCache.txt"
    "$PROJECT_ROOT/CMakeCache.txt"
    "$BUILD_DIR/CMakeFiles"
    "$PROJECT_ROOT/cmake_targets/ran_build/CMakeFiles"
)

CLEANED=0
for cache_file in "${CMAKE_CACHE_FILES[@]}"; do
    if [ -e "$cache_file" ]; then
        echo "  삭제: $cache_file"
        rm -rf "$cache_file"
        CLEANED=1
    fi
done

if [ $CLEANED -eq 0 ]; then
    echo "✓ CMake 캐시 파일이 없습니다 (정상)"
else
    echo "✓ CMake 캐시 파일 정리 완료"
fi
echo ""

# Step 3: 기타 빌드 산출물 정리
echo "[3/4] 기타 빌드 산출물 정리 중..."
# CMake 생성 파일들
find "$PROJECT_ROOT/cmake_targets/ran_build" -name "CMakeCache.txt" -o -name "cmake_install.cmake" -o -name "Makefile" 2>/dev/null | while read -r file; do
    if [ -f "$file" ]; then
        echo "  삭제: $file"
        rm -f "$file"
    fi
done

# 빌드 로그 파일 (선택적)
if [ "$1" = "--all" ] || [ "$1" = "-a" ]; then
    echo "  빌드 로그 파일도 삭제합니다..."
    rm -f "$PROJECT_ROOT/build_log*.txt" "$PROJECT_ROOT/build_*.txt" 2>/dev/null || true
fi

echo "✓ 기타 빌드 산출물 정리 완료"
echo ""

# Step 4: 최종 확인
echo "[4/4] 정리 결과 확인..."
if [ -d "$BUILD_DIR" ] || [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "⚠ 경고: 일부 파일이 남아있을 수 있습니다."
    echo "  수동으로 확인하세요: $BUILD_DIR"
else
    echo "✓ 모든 빌드 산출물이 정리되었습니다."
fi
echo ""

echo "=========================================="
echo "클린 빌드 준비 완료!"
echo "=========================================="
echo ""
echo "다음 명령어로 빌드를 시작할 수 있습니다:"
echo ""
echo "  # USRP 포함 빌드:"
echo "  ./cmake_targets/build_oai --nrUE --gNB -w SIMU -w USRP"
echo ""
echo "  # 또는 빌드 스크립트 사용:"
echo "  ./build_with_usrp.sh"
echo ""
