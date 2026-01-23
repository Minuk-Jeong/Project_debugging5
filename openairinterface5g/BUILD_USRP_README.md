# USRP 라이브러리 빌드 가이드

## 문제 원인

원본 프로젝트(Project_CSIplusMatlab)에서는 `liboai_device.so`, `liboai_usrpdevif.so`, `liboai_iqplayer.so`가 빌드 산출물로 존재하지만, 복사본 프로젝트(Project_CSIplusMatlab_13bit)에서는 `-w SIMU` 옵션만 사용하여 빌드했기 때문에 USRP 라이브러리가 빌드되지 않았습니다.

**원인:**
- `OAI_USRP:BOOL=OFF`로 설정되어 USRP 드라이버가 빌드되지 않음
- `liboai_device.so`는 `liboai_usrpdevif.so`의 심볼릭 링크로, USRP가 활성화되어야 생성됨

## 해결 방법

### 방법 1: 빌드 스크립트 사용 (권장)

```bash
cd /home/lab/바탕화면/Project_CSIplusMatlab_13bit/openairinterface5g
./build_with_usrp.sh
```

### 방법 2: 수동 빌드

```bash
cd /home/lab/바탕화면/Project_CSIplusMatlab_13bit/openairinterface5g

# Step 1: 빌드 디렉토리 정리
rm -rf cmake_targets/ran_build/build

# Step 2: USRP 활성화하여 빌드
./cmake_targets/build_oai --nrUE --gNB -w SIMU -w USRP

# Step 3: 빌드 결과 확인
find cmake_targets/ran_build/build -maxdepth 1 -name "liboai_device.so" -o -name "liboai_usrpdevif.so" -o -name "liboai_iqplayer.so"
```

## 빌드 후 확인 사항

1. **필수 라이브러리 파일 존재 확인:**
   ```bash
   cd /home/lab/바탕화면/Project_CSIplusMatlab_13bit/openairinterface5g/cmake_targets/ran_build/build
   ls -lh liboai*.so
   ```
   
   다음 파일들이 있어야 합니다:
   - `liboai_device.so` (심볼릭 링크 → `liboai_usrpdevif.so`)
   - `liboai_usrpdevif.so` (실제 라이브러리)
   - `liboai_iqplayer.so` (IQ 플레이어 라이브러리)

2. **CMakeCache 확인:**
   ```bash
   grep "OAI_USRP:BOOL" cmake_targets/ran_build/build/CMakeCache.txt
   ```
   결과: `OAI_USRP:BOOL=ON` 이어야 합니다.

3. **RUNPATH 확인:**
   ```bash
   readelf -d cmake_targets/ran_build/build/nr-uesoftmodem | grep RUNPATH
   ```
   결과: 빌드 디렉토리 경로가 포함되어 있어야 합니다.

## 실행 방법

빌드 디렉토리에서 다음 명령어로 실행 (추가 환경변수 불필요):

```bash
cd /home/lab/바탕화면/Project_CSIplusMatlab_13bit/openairinterface5g/cmake_targets/ran_build/build
sudo ./nr-uesoftmodem --usrp-args "addr=192.168.20.2,clock_source=internal,time_source=internal" \
  -O ../../../targets/PROJECTS/GENERIC-NR-5GC/CONF/nrue.uicc.conf \
  -C 3609120000 -r 51 --numerology 1 --ssb 234 --ue-nb-ant-rx 4 --ue-nb-ant-tx 1
```

## 변경 사항 요약

1. **CMakeLists.txt 수정:**
   - `nr-uesoftmodem`에 `oai_iqplayer` 의존성 추가 (`add_dependencies(nr-uesoftmodem oai_iqplayer)`)

2. **빌드 옵션 변경:**
   - 기존: `./cmake_targets/build_oai --nrUE --gNB -w SIMU`
   - 변경: `./cmake_targets/build_oai --nrUE --gNB -w SIMU -w USRP`

3. **런타임 로더:**
   - `nr-uesoftmodem`의 RUNPATH에 빌드 디렉토리가 포함되어 있어 추가 환경변수 없이 실행 가능

## 빌드 타겟 확인

빌드 로그에서 다음 타겟이 빌드되었는지 확인:
- `oai_usrpdevif` (USRP 드라이버)
- `oai_iqplayer` (IQ 플레이어)

```bash
grep -E "oai_usrpdevif|oai_iqplayer|Built target" /tmp/oai_build_usrp.log | tail -20
```
