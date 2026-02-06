# 클린 빌드 가이드

프로젝트 폴더 이름이 변경되었거나 CMake 캐시 문제가 발생할 때 사용하는 클린 빌드 가이드입니다.

## 문제 상황

프로젝트 폴더 이름이 변경되면 다음과 같은 문제가 발생할 수 있습니다:

1. **CMake 캐시에 이전 경로가 저장됨**
   - `CMakeCache.txt`에 하드코딩된 경로가 포함되어 있음
   - 빌드 시스템이 이전 경로를 참조하려고 시도

2. **상대 경로 기반 빌드 산출물 문제**
   - 일부 빌드 산출물이 이전 경로를 참조
   - 심볼릭 링크나 절대 경로가 잘못된 위치를 가리킴

3. **빌드 스크립트의 하드코딩된 경로**
   - 스크립트에 이전 프로젝트 경로가 하드코딩되어 있음

## 해결 방법

### 방법 1: 클린 빌드 스크립트 사용 (권장)

```bash
# 프로젝트 루트 디렉토리에서 실행
./clean_build.sh

# 빌드 로그 파일도 함께 삭제하려면
./clean_build.sh --all
```

이 스크립트는 다음을 수행합니다:
- 빌드 디렉토리 완전 삭제 (`cmake_targets/ran_build/build`)
- 모든 CMake 캐시 파일 정리
- CMake 생성 파일들 정리 (Makefile, cmake_install.cmake 등)
- 선택적으로 빌드 로그 파일 삭제

### 방법 2: 수동 클린 빌드

```bash
# 프로젝트 루트 디렉토리로 이동
cd "$(dirname "$0")"  # 또는 실제 프로젝트 경로

# 1. 빌드 디렉토리 삭제
rm -rf cmake_targets/ran_build/build

# 2. CMake 캐시 파일 삭제
find cmake_targets/ran_build -name "CMakeCache.txt" -delete
find cmake_targets/ran_build -name "CMakeFiles" -type d -exec rm -rf {} + 2>/dev/null || true

# 3. CMake 생성 파일 삭제
find cmake_targets/ran_build -name "cmake_install.cmake" -delete
find cmake_targets/ran_build -name "Makefile" -delete

# 4. (선택) 빌드 로그 삭제
rm -f build_log*.txt build_*.txt
```

### 방법 3: 완전한 클린 빌드 (모든 빌드 산출물 삭제)

```bash
# 프로젝트 루트 디렉토리에서
rm -rf cmake_targets/ran_build/build
rm -rf cmake_targets/ran_build/build_test
find . -name "CMakeCache.txt" -delete
find . -name "CMakeFiles" -type d -exec rm -rf {} + 2>/dev/null || true
find . -name "*.cmake" -path "*/cmake_targets/*" -delete
find . -name "Makefile" -path "*/cmake_targets/*" -delete
```

## 클린 빌드 후 빌드 방법

클린 빌드를 수행한 후, 다음 명령어로 빌드를 시작합니다:

### USRP 포함 빌드

```bash
# 방법 1: 빌드 스크립트 사용
./build_with_usrp.sh

# 방법 2: 직접 빌드
./cmake_targets/build_oai --nrUE --gNB -w SIMU -w USRP
```

### 일반 빌드

```bash
./cmake_targets/build_oai --nrUE --gNB -w SIMU
```

## 프로젝트 폴더 이름 변경 시 체크리스트

프로젝트 폴더 이름을 변경한 경우 다음을 확인하세요:

1. ✅ **빌드 스크립트 경로 확인**
   - 모든 스크립트가 자동으로 경로를 감지하도록 수정됨
   - 하드코딩된 경로가 없음

2. ✅ **CMake 캐시 정리**
   - `./clean_build.sh` 실행
   - 또는 수동으로 CMakeCache.txt 삭제

3. ✅ **빌드 디렉토리 정리**
   - `cmake_targets/ran_build/build` 디렉토리 삭제

4. ✅ **새로운 경로에서 빌드**
   - 클린 빌드 후 새 경로에서 빌드 시작

## 주의사항

1. **빌드 산출물 백업**
   - 클린 빌드를 수행하면 모든 빌드 산출물이 삭제됩니다
   - 필요한 바이너리나 라이브러리는 미리 백업하세요

2. **빌드 시간**
   - 클린 빌드 후 처음 빌드하는 경우 시간이 오래 걸릴 수 있습니다
   - ccache를 사용하면 재빌드 시간을 단축할 수 있습니다

3. **의존성 확인**
   - 클린 빌드 후 의존성 설치가 필요할 수 있습니다
   - `./cmake_targets/build_oai -I`로 의존성을 설치할 수 있습니다

## 문제 해결

### CMake 캐시가 계속 남아있는 경우

```bash
# 강제로 모든 CMake 관련 파일 삭제
find . -name "CMakeCache.txt" -exec rm -f {} \;
find . -name "CMakeFiles" -type d -exec rm -rf {} + 2>/dev/null || true
```

### 빌드 스크립트가 이전 경로를 참조하는 경우

모든 빌드 스크립트는 자동으로 경로를 감지하도록 수정되었습니다. 
만약 문제가 발생하면 스크립트의 다음 부분을 확인하세요:

```bash
# 스크립트 위치를 기준으로 프로젝트 루트 자동 감지
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
```

이 코드가 모든 스크립트에 포함되어 있어야 합니다.

## 관련 파일

- `clean_build.sh`: 클린 빌드 스크립트
- `build_with_usrp.sh`: USRP 포함 빌드 스크립트
- `rebuild_with_usrp.sh`: 완전한 재빌드 스크립트
- `CMakeLists.txt`: CMake 빌드 설정 (경로 자동 감지)
