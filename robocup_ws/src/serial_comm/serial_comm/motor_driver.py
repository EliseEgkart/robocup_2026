#!/usr/bin/env python3
#=====================================================#
# 기능: MDrobot MDH100, MD200T Motor Driver
# - 모터굴리기..!
# - 일단 귀찮으니 대충 적어놓고,,
#
# TODO : 할거 많음..
# 최종 수정일: 2025.11.09
# 편집자: 김형진
#=====================================================#
import time
import numpy as np
import serial
from serial_comm.helper import Helper

'''
< 송신 데이터 포맷 (PC → MotorDriver) >
┌──────┬──────┬────────────┬────────┬──────┬──────────┬──────────────┬──────────────┬──────┐
│ RMID │ TMID │ Driver ID  │  PID   │ LEN  │   DATA   │     ...      │     ...      │ CHK  │
├──────┼──────┼────────────┼────────┼──────┼──────────┼──────────────┼──────────────┼──────┤
│ 183  │ 184  │     1      │ 0~255  │  N   │  값(0~255)│  ...payload │  ...payload │ 계산값 │
└──────┴──────┴────────────┴────────┴──────┴──────────┴──────────────┴──────────────┴──────┘

설명:
 - RMID : 송신자 ID (Master, PC)
 - TMID : 수신자 ID (Slave, Motor Driver)
 - Driver ID : 제어 대상 드라이버 번호
 - PID : 명령 코드 (예: 속도 제어, 상태 요청 등)
 - LEN : DATA 구간의 바이트 길이
 - DATA : 명령 데이터 (예: 목표 속도, 방향 등)
 - CHK : 체크섬 (모든 바이트의 합을 1바이트로 더한 뒤 보수 + 1)

예시:
 [183, 184, 1, 1, 4, 0, 100, 0, 200, 58]
 RMID | TMID | ID | PID | LEN | DATA0 | DATA1 | DATA2 | DATA3 | CHK


< 수신 데이터 포맷 (MotorDriver → PC) >
┌──────┬──────┬────────────┬────────┬──────┬──────────┬──────────────┬──────────────┬──────┐
│ RMID │ TMID │ Driver ID  │  PID   │ LEN  │   DATA   │     ...      │     ...      │ CHK  │
├──────┼──────┼────────────┼────────┼──────┼──────────┼──────────────┼──────────────┼──────┤
│ 184  │ 183  │     1      │ 0~255  │  N   │  값(0~255)│  ...payload │  ...payload │ 계산값 │
└──────┴──────┴────────────┴────────┴──────┴──────────┴──────────────┴──────────────┴──────┘

설명:
 - RMID : 송신자 ID (Slave, Motor Driver)
 - TMID : 수신자 ID (Master, PC)
 - Driver ID : 응답한 드라이버 번호
 - PID : 응답 코드 (예: 상태 값, 에러 등)
 - LEN : DATA 구간의 바이트 길이
 - DATA : 상태 데이터 (예: 속도, 전류, 엔코더 등)
 - CHK : 체크섬 (모든 바이트의 합을 1바이트로 더한 뒤 보수 + 1)

예시:
 [184, 183, 1, 129, 6, 0, 30, 0, 2, 0, 60, 177]
 RMID | TMID | ID | PID | LEN | DATA0 | DATA1 | DATA2 | DATA3 | DATA4 | DATA5 | CHK
'''

class MotorDriver:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB_RS485', 115200, timeout=0.5)
        self.RMID = 183
        self.TMID = 184
        self.driverID = 1

        self.encoder_gain = 250      # encoder change value over 360 degree
        self.rpm1, self.rpm2 = 0, 0
        self.current1, self.current2 = 0., 0.
        self.status1, self.status2 = 0, 0
        self.pos1, self.pos2 = 0, 0

        # =======================
        # DEBUG / STATS
        # =======================
        self.DEBUG_RX = True                 # 원인 로그 켜기
        self.DEBUG_RX_RAW = True             # raw hex 덤프 켜기
        self.DEBUG_RX_PERIOD_SEC = 0.2       # 로그 출력 최소 주기(초) (너무 많으면 0.5~1.0으로)
        self._dbg_last_print_t = time.time()

        self.rx_ok = 0
        self.rx_timeout = 0
        self.rx_bad_len = 0
        self.rx_invalid = 0
        self.rx_last_ok_t = None

        # 값 변화 감지용(옵션)
        self._last_pos1 = None
        self._last_pos2 = None
        self._last_rpm1 = None
        self._last_rpm2 = None

    def version_check(self):
        pid = 1
        byChkSend = np.array((self.RMID + self.TMID + 1 + 4 + 1 + pid) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, 4, 1, pid, chk], dtype=np.uint8)
        print(data)
        self.ser.write(data.tobytes())

        readbytes = self.ser.read(size=7)
        received_data = np.frombuffer(readbytes, dtype=np.uint8)
        print(received_data)

    def send_torque_cmd(self, t1, t2):
        pid = 209
        datanum = 7
        t1_ = np.array(Helper.int16_to_uint8arr(np.array(t1, dtype=np.int16)), dtype=np.uint8)
        t2_ = np.array(Helper.int16_to_uint8arr(np.array(-t2, dtype=np.int16)), dtype=np.uint8)
        data = np.array(1, dtype=np.uint8)
        data = np.append(data, t1_)
        data = np.append(data, np.array(1, dtype=np.uint8))
        data = np.append(data, t2_)
        data = np.append(data, np.array(2, dtype=np.uint8))

        send_data = np.array([self.RMID, self.TMID, self.driverID, pid, datanum], dtype=np.uint8)
        send_data = np.append(send_data, data)
        byChkSend = np.array(np.sum(send_data, dtype=np.uint8))
        chk = np.array(~byChkSend + 1, dtype=np.uint8)
        send_data = np.append(send_data, chk)

        self.ser.write(send_data.tobytes())

    def recv_motor_state(self):
        """
        상태 프레임(24B) 수신.
        - 현재 구현은 프레임 경계가 항상 맞는다고 가정해서, 종종 invalid frame이 날 수 있음.
        - 여기서는 '원인별 로그 + raw dump + 통계'를 넣어서 실제로 뭐가 깨지는지 잡는다.
        """
        FRAME_SIZE = 24
        readbytes = self.ser.read(size=FRAME_SIZE)
        data = np.frombuffer(readbytes, dtype=np.uint8)

        now_t = time.time()
        do_print = self.DEBUG_RX and ((now_t - self._dbg_last_print_t) >= self.DEBUG_RX_PERIOD_SEC)

        # 1) 타임아웃/길이 불일치
        if len(data) == 0:
            self.rx_timeout += 1
            if do_print:
                self._dbg_last_print_t = now_t
                print(f"[recv_motor_state][TIMEOUT] read=0B ok={self.rx_ok} invalid={self.rx_invalid} bad_len={self.rx_bad_len} timeout={self.rx_timeout}")
            return False

        if len(data) != FRAME_SIZE:
            self.rx_bad_len += 1
            if do_print:
                self._dbg_last_print_t = now_t
                if self.DEBUG_RX_RAW:
                    raw_hex = " ".join([f"{b:02X}" for b in data.tolist()])
                    print(f"[recv_motor_state][BAD_LEN] read={len(data)}B expected={FRAME_SIZE}B raw=({raw_hex})")
                else:
                    print(f"[recv_motor_state][BAD_LEN] read={len(data)}B expected={FRAME_SIZE}B")
            return False

        # 2) 프레임 필드 검증
        exp0 = self.TMID       # data[0] should be 184
        exp1 = self.RMID       # data[1] should be 183
        exp2 = self.driverID   # data[2] should be 1
        exp3 = 210             # PID
        exp4 = 18              # LEN
        sum_u8 = int(np.sum(data, dtype=np.uint8))  # 체크섬 포함 sum(uint8) == 0 기대

        bad_reasons = []
        if int(data[0]) != exp0: bad_reasons.append(f"data[0]={int(data[0])} exp={exp0} (RMID/TMID swapped?)")
        if int(data[1]) != exp1: bad_reasons.append(f"data[1]={int(data[1])} exp={exp1}")
        if int(data[2]) != exp2: bad_reasons.append(f"data[2]={int(data[2])} exp={exp2}")
        if int(data[3]) != exp3: bad_reasons.append(f"data[3]={int(data[3])} exp={exp3} (PID)")
        if int(data[4]) != exp4: bad_reasons.append(f"data[4]={int(data[4])} exp={exp4} (LEN)")
        if sum_u8 != 0:          bad_reasons.append(f"CHK sum(uint8)={sum_u8} (should be 0)")

        if bad_reasons:
            self.rx_invalid += 1
            if do_print:
                self._dbg_last_print_t = now_t
                if self.DEBUG_RX_RAW:
                    raw_hex = " ".join([f"{b:02X}" for b in data.tolist()])
                    print(f"[recv_motor_state][INVALID] raw=({raw_hex})")
                print("[recv_motor_state][INVALID] reasons:")
                for r in bad_reasons:
                    print(f"  - {r}")
                print(f"[recv_motor_state][STATS] ok={self.rx_ok} invalid={self.rx_invalid} bad_len={self.rx_bad_len} timeout={self.rx_timeout}")
            return False

        # 3) 정상 프레임 파싱
        self.rpm1 = -Helper.uint8arr_to_int16(data[5], data[6])
        self.current1 = Helper.uint8arr_to_int16(data[7], data[8])
        self.status1 = int(data[9])
        self.pos1 = -Helper.uint8arr_to_int32(data[10], data[11], data[12], data[13])

        self.rpm2 = Helper.uint8arr_to_int16(data[14], data[15])
        self.current2 = Helper.uint8arr_to_int16(data[16], data[17])
        self.status2 = int(data[18])
        self.pos2 = Helper.uint8arr_to_int32(data[19], data[20], data[21], data[22])

        self.rx_ok += 1
        self.rx_last_ok_t = now_t

        # 4) "값이 바뀔 때만" 별도 로그(너가 원하는 모터 raw 값 관찰)
        changed = False
        if self._last_pos1 is None:
            changed = True
        else:
            if self.pos1 != self._last_pos1: changed = True
            if self.pos2 != self._last_pos2: changed = True
            if self.rpm1 != self._last_rpm1: changed = True
            if self.rpm2 != self._last_rpm2: changed = True

        self._last_pos1 = self.pos1
        self._last_pos2 = self.pos2
        self._last_rpm1 = self.rpm1
        self._last_rpm2 = self.rpm2

        if self.DEBUG_RX and do_print and changed:
            self._dbg_last_print_t = now_t
            print(f"[recv_motor_state][OK] rpm1={self.rpm1:6d} rpm2={self.rpm2:6d} "
                  f"pos1={self.pos1:10d} pos2={self.pos2:10d} "
                  f"cur1={int(self.current1):6d} cur2={int(self.current2):6d} "
                  f"st1={self.status1:3d} st2={self.status2:3d} "
                  f"(ok={self.rx_ok} invalid={self.rx_invalid})")

        return True

    def recv_watch_delay(self): # PID_COM_WATCH_DELAY
        pid = 185
        byChkSend = np.array((self.RMID + self.TMID + 1 + 4 + 1 + pid) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, 4, 1, pid, chk], dtype=np.uint8)
        print(data)
        self.ser.write(data.tobytes())

        readbytes = self.ser.read(size=8)
        received_data = np.frombuffer(readbytes, dtype=np.uint8)
        print(received_data)

    def recv_stop_status(self): # PID_STOP_STATUS
        pid = 24
        new_stop_status = 1     # Default = 1

        byChkSend = np.array((self.RMID + self.TMID + 1 + 4 + 1 + pid) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, 4, 1, pid, chk], dtype=np.uint8)
        self.ser.write(data.tobytes())

        readbytes = self.ser.read(size=7)
        received_data = np.frombuffer(readbytes, dtype=np.uint8)
        if len(received_data) == 7:
            current_stop_status = received_data[5]
        else:
            current_stop_status = new_stop_status

        if new_stop_status != current_stop_status:
            byChkSend = np.array((self.RMID + self.TMID + 1 + pid + 1 + new_stop_status) % 256, dtype=np.uint8)
            chk = ~byChkSend + 1
            data = np.array([self.RMID, self.TMID, 1, pid, 1, new_stop_status, chk], dtype=np.uint8)
            self.ser.write(data.tobytes())
            print("stop status changed from " + str(current_stop_status) + " to " + str(new_stop_status) + ".")
        else:
            print(received_data)

    def recv_read_this(self):
        pid = 135
        data_size = 2
        byChkSend = np.array((self.RMID + self.TMID + 1 + 4 + 1 + pid) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, 4, 1, pid, chk], dtype=np.uint8)
        print(data)
        self.ser.write(data.tobytes())

        readbytes = self.ser.read(size=data_size+6)
        received_data = np.frombuffer(readbytes, dtype=np.uint8)
        print(received_data)

    def write_BAUD(self):
        pid = 135
        new_BAUD = 4        # Default = 2

        data_size = 2
        byChkSend = np.array((self.RMID + self.TMID + 1 + pid + data_size + 0xAA + new_BAUD) % 256, dtype=np.uint8)
        chk = ~byChkSend + 1
        data = np.array([self.RMID, self.TMID, 1, pid, data_size, 0xAA, new_BAUD, chk], dtype=np.uint8)

        print("BAUDRATE UPDATED!")
        self.ser.write(data.tobytes())

    def send_position_cmd(self, p1, p2, mv1, mv2):
        pid = 206
        datanum = 15
        p1_ = np.array(Helper.int32_to_uint8arr(np.array(-p1, dtype=np.int32)), dtype=np.uint8)
        mv1_ = np.array(Helper.int16_to_uint8arr(np.array(mv1, dtype=np.int16)), dtype=np.uint8)
        p2_ = np.array(Helper.int32_to_uint8arr(np.array(p2, dtype=np.int32)), dtype=np.uint8)
        mv2_ = np.array(Helper.int16_to_uint8arr(np.array(mv2, dtype=np.int16)), dtype=np.uint8)
        data = np.array(1, dtype=np.uint8)
        data = np.append(data, p1_)
        data = np.append(data, mv1_)
        data = np.append(data, np.array(1, dtype=np.uint8))
        data = np.append(data, p2_)
        data = np.append(data, mv2_)
        data = np.append(data, np.array(2, dtype=np.uint8))

        send_data = np.array([self.RMID, self.TMID, self.driverID, pid, datanum], dtype=np.uint8)
        send_data = np.append(send_data, data)
        byChkSend = np.array(np.sum(send_data, dtype=np.uint8))
        chk = np.array(~byChkSend + 1, dtype=np.uint8)
        send_data = np.append(send_data, chk)

        self.ser.write(send_data.tobytes())

    def send_vel_cmd(self, v1, v2):
        pid = 207
        datanum = 7
        v1_ = np.array(Helper.int16_to_uint8arr(np.array(-v1, dtype=np.int16)), dtype=np.uint8)
        v2_ = np.array(Helper.int16_to_uint8arr(np.array(v2, dtype=np.int16)), dtype=np.uint8)
        data = np.array(1, dtype=np.uint8)
        data = np.append(data, v1_)
        data = np.append(data, np.array(1, dtype=np.uint8))
        data = np.append(data, v2_)
        data = np.append(data, np.array(2, dtype=np.uint8))

        send_data = np.array([self.RMID, self.TMID, self.driverID, pid, datanum], dtype=np.uint8)
        send_data = np.append(send_data, data)
        byChkSend = np.array(np.sum(send_data, dtype=np.uint8))
        chk = np.array(~byChkSend + 1, dtype=np.uint8)
        send_data = np.append(send_data, chk)

        self.ser.write(send_data.tobytes())
