Project_debugging5/openairinterface5g 경로에서 ./build_with_usrp.sh 를 실행하세요.
이후 cd ran_build/build 입력 후 
sudo env \
  OAI_DL_CSI_ONNX_MODEL="/home/lab/바탕화면/Project_debugging5/openairinterface5g/models/channel_predictor.onnx" \
  OAI_DL_CSI_ONNX_APPLY=1 \
  ./nr-uesoftmodem --usrp-args "addr=192.168.20.2, clock_source=internal,time_source=internal" -O ../../../targets/PROJECTS/GENERIC-NR-5GC/CONF/nrue.uicc.conf -C 3609120000 -r 51 --numerology 1 --ssb 234 --ue-nb-ant-rx 4 --ue-nb-ant-tx 1 --onnx 0
  를 입력하시면 됩니다. 줄 바꿈에 의해 IP 주소가 깨지지 않는지, onnx = 0 or 1 원하는 게 잘 선택되었는지 확인하세요.
