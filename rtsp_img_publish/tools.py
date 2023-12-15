import numpy as np
import cv2

def get_nalu_pos(byte_stream):
    size = byte_stream.__len__()
    nals = []
    retnals = []
    startCodePrefixShort = b"\x00\x00\x01"
    pos = 0
    while pos < size:
        is4bytes = False
        retpos = byte_stream.find(startCodePrefixShort, pos)
        if retpos == -1:
            break
        if byte_stream[retpos - 1] == 0:
            retpos -= 1
            is4bytes = True
        if is4bytes:
            pos = retpos + 4
        else:
            pos = retpos + 3
        val = hex(byte_stream[pos])
        val = "{:d}".format(byte_stream[pos], 4)
        val = int(val)
        fb = (val >> 7) & 0x1
        nri = (val >> 5) & 0x3
        type = val & 0x1f
        nals.append((pos, is4bytes, fb, nri, type))
    for i in range(0, len(nals) - 1):
        start = nals[i][0]
        if nals[i + 1][1]:
            end = nals[i + 1][0] - 5
        else:
            end = nals[i + 1][0] - 4
        retnals.append((start, end, nals[i][1], nals[i][2], nals[i][3], nals[i][4]))
    start = nals[-1][0]
    end = byte_stream.__len__() - 1
    retnals.append((start, end, nals[-1][1], nals[-1][2], nals[-1][3], nals[-1][4]))
    return retnals

def get_h264_nalu_type(byte_stream):
    nalu_types = []
    nalu_pos = get_nalu_pos(byte_stream)
    for idx, (start, end, is4bytes, fb, nri, type) in enumerate(nalu_pos):
        # print("NAL#%d: %d, %d, %d, %d, %d" % (idx, start, end, fb, nri, type))
        nalu_types.append(type)
    return nalu_types

def is_pps_sps(byte_stream):
    nalu_types = get_h264_nalu_type(byte_stream)
    if nalu_types[0] in [1, 5]:
        return False
    else:
        return True

def nv12_bgr(nv12_img_bytes, width, height):
    frame = np.frombuffer(nv12_img_bytes, dtype = np.uint8)
    img_bgr = cv2.cvtColor(frame.reshape((height *3 // 2, width)), cv2.COLOR_YUV2BGR_NV12)
    return img_bgr