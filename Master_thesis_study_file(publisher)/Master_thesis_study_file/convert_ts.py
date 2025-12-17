#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fastdds_udp_tx.log → *.conv (텍스트 2줄/패킷: 메타라인 + hex라인)

사용:
  python3 /tmp/convert_ts.py <입력로그경로> --outfile <출력파일경로>

예:
  python3 /tmp/convert_ts.py fastdds_udp_tx.log --outfile con123.conv

환경변수:
  TZ_OFFSET_HOURS : 시각 오프셋 시간(기본 9=KST), 0 으로 두면 UTC
  LIMIT           : 최대 처리 패킷 수(정수, 0=무제한)
  DELETE_INPUT    : 변환 성공 후 입력 로그 삭제(1이면 삭제)
"""

import os
import re
import sys
import datetime

META_RE = re.compile(
    r"^ts_us=(?P<ts_us>\d+)\s+src=(?P<src>\S+)\s+dst=(?P<dst>\S+)\s+bytes=(?P<bytes>\d+)\s+rtps=(?P<rtps>\S+)(?:\s+discovery=(?P<disc>\d+))?"
)
HEX_RE = re.compile(r"^hex=(?P<hex>[0-9A-Fa-f\s]+)$")

def eprint(*a, **k): print(*a, file=sys.stderr, **k)

def kst_from_ts_us(ts_us: int, offset_hours: int = 9) -> str:
    dt_utc = datetime.datetime.utcfromtimestamp(ts_us / 1e6)
    dt = dt_utc + datetime.timedelta(hours=offset_hours)
    ms = dt.microsecond // 1000
    return dt.strftime(f"%Y-%m-%d %H:%M:%S.{ms:03d}")

def main():
    # -------- 인자 파싱(단일 입력 + --outfile 필수) --------
    if len(sys.argv) < 2 or "--outfile" not in sys.argv:
        eprint("사용법: python3 /tmp/convert_ts.py <입력로그> --outfile <출력파일>")
        sys.exit(2)

    # 입력 파일은 첫 번째 비옵션 인자로 간주
    argv = [a for a in sys.argv[1:] if a != "--"]
    input_path = None
    outfile = None

    # 첫 번째 토큰이 입력경로, '--outfile' 값은 다음 토큰
    i = 0
    while i < len(argv):
        tok = argv[i]
        if tok == "--outfile":
            if i + 1 >= len(argv):
                eprint("[ERR] --outfile 값이 없습니다.")
                sys.exit(2)
            outfile = argv[i + 1]
            i += 2
            continue
        if input_path is None and not tok.startswith("-"):
            input_path = tok
            i += 1
            continue
        # 기타 옵션/토큰은 무시
        i += 1

    if not input_path:
        eprint("[ERR] 입력 로그 경로를 지정하세요.")
        sys.exit(2)
    if not outfile:
        eprint("[ERR] --outfile <출력파일> 을 지정하세요.")
        sys.exit(2)

    try:
        tz_off = int(os.environ.get("TZ_OFFSET_HOURS", "9"))
    except ValueError:
        tz_off = 9

    try:
        limit = int(os.environ.get("LIMIT", "0"))
    except ValueError:
        limit = 0

    if not os.path.exists(input_path):
        eprint(f"[ERR] 입력 로그를 찾을 수 없습니다: {input_path}")
        sys.exit(1)

    out_dir = os.path.dirname(os.path.abspath(outfile)) or "."
    os.makedirs(out_dir, exist_ok=True)

    total = 0
    written = 0
    pending = False  # 직전 메타를 받았는지

    with open(input_path, "r", encoding="utf-8", errors="ignore") as fin, \
         open(outfile,   "w", encoding="utf-8") as fout:

        for raw in fin:
            line = raw.strip()
            if not line:
                continue

            mm = META_RE.match(line)
            if mm:
                g = mm.groupdict()
                ts_us = int(g["ts_us"])
                ts_txt = kst_from_ts_us(ts_us, tz_off)
                # 메타라인(사람이 읽기 쉬운 ts 추가)
                meta = (
                    f"ts={ts_txt} "
                    f"ts_us={g['ts_us']} src={g['src']} dst={g['dst']} "
                    f"bytes={g['bytes']} rtps={g['rtps']}"
                )
                if g.get("disc"):
                    meta += f" discovery={g['disc']}"
                fout.write(meta + "\n")
                total += 1
                pending = True
                continue

            if pending:
                mh = HEX_RE.match(line)
                if mh:
                    # 공백 제거만 수행(HEX 본문 정규화)
                    hextext = "".join(ch for ch in mh.group("hex")
                                      if ch in "0123456789abcdefABCDEF")
                    fout.write(f"hex={hextext}\n")
                    written += 1
                    pending = False
                    if limit and written >= limit:
                        break
                # hex 라인이 아니면 무시

    eprint(f"[OK] parsed={total}, written={written}, input='{input_path}', output='{outfile}'")

    if os.environ.get("DELETE_INPUT") == "1":
        try:
            os.remove(input_path)
            eprint(f"[OK] deleted input: {input_path}")
        except Exception as ex:
            eprint(f"[WARN] failed to delete input: {ex}")

if __name__ == "__main__":
    main()

