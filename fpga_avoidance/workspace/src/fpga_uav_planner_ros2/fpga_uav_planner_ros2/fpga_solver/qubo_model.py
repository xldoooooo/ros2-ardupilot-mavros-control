from __future__ import annotations
from dataclasses import dataclass
import binascii,json,math,struct,zipfile
import numpy as np
from pathlib import Path
from typing import List,Optional,Sequence
from protocol import FORMAT_Q9_22,FORMAT_BFP16_J,FORMAT_BFP8_J,FORMAT_UAV_BFP8_J,FORMAT_UAV_BFP16_J,FORMAT_UAV_Q9_22,FORMAT_UAV_SPARSE_BFP16_J,FLAG_UPPER_TRIANGLE

N_HW=256; FRAC_BITS=22; SCALE=1<<FRAC_BITS; Q_MIN=-(1<<31); Q_MAX=(1<<31)-1; UPPER_COUNT=N_HW*(N_HW-1)//2
LOAD_PAYLOAD_SIZE_Q9=4+N_HW*4+UPPER_COUNT*4
LOAD_PAYLOAD_SIZE_BFP16J=4+N_HW*4+UPPER_COUNT*2
LOAD_PAYLOAD_SIZE_BFP8J=4+N_HW*4+UPPER_COUNT
LOAD_PAYLOAD_SIZE=LOAD_PAYLOAD_SIZE_Q9
_UPPER_I, _UPPER_J = np.triu_indices(N_HW, 1)

def encode_size_code(n:int)->int:
    if not 0 <= int(n) <= 256: raise ValueError(f"规模编码超界: {n}")
    return 0 if int(n)==256 else int(n)

def decode_size_code(code:int)->int:
    code=int(code)&0xFF
    return 256 if code==0 else code
@dataclass
class Problem:
    name:str; kind:str; n:int; h:List[float]; j:List[List[float]]; source_matrix:List[List[float]]; metadata:dict; fpga_scale:float=1.0
    def validate(self):
        # Problems are immutable after construction in this application. Cache the
        # validation result because payload construction / repeated FPGA attempts
        # used to rescan all 32,640 couplings several times per solve.
        if getattr(self, "_validated", False):
            return
        if not 2<=self.n<=N_HW: raise ValueError("问题规模必须在2到256之间")
        if len(self.h)!=N_HW or len(self.j)!=N_HW or any(len(r)!=N_HW for r in self.j): raise ValueError("内部Ising必须为256维")
        h=np.asarray(self.h,dtype=np.float64)
        j=np.asarray(self.j,dtype=np.float64)
        if h.shape!=(N_HW,) or j.shape!=(N_HW,N_HW): raise ValueError("内部Ising必须为256维")
        if not np.isfinite(h).all() or not np.isfinite(j).all(): raise ValueError("系数非有限")
        if np.max(np.abs(np.diag(j)))>1e-12: raise ValueError("J对角线必须为0")
        if np.max(np.abs(j-j.T))>1e-9: raise ValueError("J必须对称")
        vals=np.concatenate((h,j[_UPPER_I,_UPPER_J]))
        raw=np.rint(vals*float(SCALE))
        if raw.size and (raw.min()<Q_MIN or raw.max()>Q_MAX): raise ValueError("系数超出Q9.22范围")
        self._validated=True
def quantize_q9_22(v:float)->int:
    if not math.isfinite(float(v)): raise ValueError("系数非有限")
    r=int(round(float(v)*SCALE))
    if not Q_MIN<=r<=Q_MAX: raise ValueError(f"系数{v}超出Q9.22范围")
    return r
def dequantize_q9_22(r:int)->float:
    r&=0xFFFFFFFF
    if r&0x80000000:r-=1<<32
    return r/SCALE
def blank():return [[0.0]*N_HW for _ in range(N_HW)]
def pad(m,n):
    o=blank()
    for i in range(n):
        for j in range(n):o[i][j]=float(m[i][j])
    return o
def _normalize_for_fpga(h,j,n,target_energy=240.0,max_coefficient=64.0,as_numpy=False,inplace=False):
    """Vectorized positive rescaling preserving the optimum.

    ``inplace=True`` is used only for freshly allocated online QUBO arrays and
    avoids two full padded-array copies on the Jetson hot path.
    """
    if inplace:
        ha=np.asarray(h,dtype=np.float64)
        ja=np.asarray(j,dtype=np.float64)
    else:
        ha=np.array(h,dtype=np.float64,copy=True)
        ja=np.array(j,dtype=np.float64,copy=True)
    hn=np.abs(ha[:n])
    iu,ju=np.triu_indices(n,1)
    jn=np.abs(ja[:n,:n][iu,ju]) if n>1 else np.empty(0,dtype=np.float64)
    bound=float(hn.sum()+2.0*jn.sum())
    max_abs=float(max(hn.max(initial=0.0),jn.max(initial=0.0)))
    scale=1.0
    if bound>target_energy and bound>0.0:scale=min(scale,target_energy/bound)
    if max_abs>max_coefficient and max_abs>0.0:scale=min(scale,max_coefficient/max_abs)
    if scale != 1.0:
        if inplace:
            ha[:n] *= scale
            ja[:n,:n] *= scale
        else:
            ha *= scale
            ja *= scale
    nz=np.concatenate((np.abs(ha[:n][ha[:n]!=0.0]),np.abs(ja[:n,:n][iu,ju][ja[:n,:n][iu,ju]!=0.0])))
    quantized_zero=int(np.count_nonzero(np.rint(nz*float(SCALE))==0)) if nz.size else 0
    if as_numpy:
        return ha,ja,scale,{"energy_abs_bound_before_scale":bound,"energy_abs_bound_after_scale":bound*scale,"max_coefficient_before_scale":max_abs,"fpga_coefficient_scale":scale,"quantized_to_zero_count":quantized_zero}
    return ha.tolist(),ja.tolist(),scale,{"energy_abs_bound_before_scale":bound,"energy_abs_bound_after_scale":bound*scale,"max_coefficient_before_scale":max_abs,"fpga_coefficient_scale":scale,"quantized_to_zero_count":quantized_zero}
def parse_numeric_matrix(text:str,n:int):
    t=text.strip(); rows=json.loads(t) if t.startswith("[") else [[float(x) for x in l.replace(","," ").split()] for l in t.splitlines() if l.strip() and not l.lstrip().startswith("#")]
    if isinstance(rows,dict):rows=rows.get("matrix",rows.get("qubo",rows.get("adjacency")))
    if len(rows)!=n or any(len(r)!=n for r in rows):raise ValueError(f"需要{n}×{n}矩阵")
    return [[float(x) for x in r] for r in rows]
def parse_edge_list(text,n):
    m=[[0.0]*n for _ in range(n)]; c=0
    for raw in text.splitlines():
        p=raw.replace(","," ").split()
        if not p or raw.lstrip().startswith("#"):continue
        if len(p) not in(2,3):raise ValueError(f"边格式错误: {raw}")
        u,v=int(p[0]),int(p[1]);w=float(p[2]) if len(p)==3 else 1.0
        if not(0<=u<n and 0<=v<n) or u==v:raise ValueError(f"非法边{u},{v}")
        m[u][v]+=w;m[v][u]+=w;c+=1
    if not c:raise ValueError("没有边")
    return m
def make_maxcut(a,n,name="maxcut"):
    h0=[0.0]*N_HW;j0=blank()
    for i in range(n):
        if abs(a[i][i])>1e-12:raise ValueError("邻接矩阵对角线必须为0")
        for k in range(i+1,n):
            if abs(a[i][k]-a[k][i])>1e-9:raise ValueError("邻接矩阵必须对称")
            j0[i][k]=j0[k][i]=-0.5*float(a[i][k])
    h,j,scale,norm=_normalize_for_fpga(h0,j0,n)
    meta={"mapping":"J=-W/2,h=0",**norm}
    p=Problem(name,"maxcut",n,h,j,pad(a,n),meta,scale);p.validate();return p

def make_qubo(q,n,name="qubo",max_energy=240.0,keep_numpy=False):
    qa=np.asarray(q,dtype=np.float64)
    if qa.shape!=(n,n): raise ValueError(f"QUBO矩阵应为{n}×{n}")
    if not np.isfinite(qa).all(): raise ValueError("QUBO系数非有限")
    # x^TQx depends on Q_ij+Q_ji for i!=j. Building the full off-diagonal
    # symmetric matrix directly avoids the previous triu -> triu -> transpose
    # allocation chain while preserving identical algebra.
    diag_idx=np.arange(n,dtype=np.intp)
    off_sym=qa+qa.T
    off_sym[diag_idx,diag_idx]=0.0
    hsmall=-(qa[diag_idx,diag_idx]+0.5*off_sym.sum(axis=1))
    jsmall=-0.25*off_sym
    h0=np.zeros(N_HW,dtype=np.float64); h0[:n]=hsmall
    j0=np.zeros((N_HW,N_HW),dtype=np.float64); j0[:n,:n]=jsmall
    h,j,scale,norm=_normalize_for_fpga(
        h0,j0,n,target_energy=float(max_energy),as_numpy=bool(keep_numpy),inplace=bool(keep_numpy)
    )
    source = qa.copy() if keep_numpy else qa.tolist()
    p=Problem(name,"qubo",n,h,j,source,{"objective":"x^TQx",**norm},scale)
    p._validated=True
    return p

def make_qubo_from_terms(linear, quadratic, n, name="qubo", max_energy=240.0, keep_numpy=False):
    """Build the identical Ising mapping directly from QUBO linear/off-diagonal terms.

    ``quadratic`` is the symmetric zero-diagonal matrix used by the online
    accumulator, while ``linear`` is the QUBO diagonal.  This avoids rebuilding
    Q, forming Q+Q.T, and then discarding that temporary matrix again.
    """
    linear=np.asarray(linear,dtype=np.float64)
    quad=np.asarray(quadratic,dtype=np.float64)
    if linear.shape!=(n,) or quad.shape!=(n,n): raise ValueError("QUBO terms shape mismatch")
    if not np.isfinite(linear).all() or not np.isfinite(quad).all(): raise ValueError("QUBO系数非有限")
    # For symmetric zero-diagonal Q_off: off_sym = Q+Q.T = 2*Q_off.
    hsmall=-(linear+quad.sum(axis=1))
    jsmall=-0.5*quad
    h0=np.zeros(N_HW,dtype=np.float64); h0[:n]=hsmall
    j0=np.zeros((N_HW,N_HW),dtype=np.float64); j0[:n,:n]=jsmall
    h,j,scale,norm=_normalize_for_fpga(
        h0,j0,n,target_energy=float(max_energy),as_numpy=bool(keep_numpy),inplace=bool(keep_numpy)
    )
    source=quad.copy()
    source[np.arange(n,dtype=np.intp),np.arange(n,dtype=np.intp)]=linear
    if not keep_numpy: source=source.tolist()
    p=Problem(name,"qubo",n,h,j,source,{"objective":"x^TQx",**norm},scale)
    p._validated=True
    return p

def make_ising(hv,jv,n,name="ising"):
    h0=[0.0]*N_HW;j0=blank()
    if len(hv)!=n:raise ValueError("h长度错误")
    for i in range(n):
        h0[i]=float(hv[i])
        for k in range(i+1,n):
            if abs(float(jv[i][k])-float(jv[k][i]))>1e-9:raise ValueError("J必须对称")
            j0[i][k]=j0[k][i]=float(jv[i][k])
    h,j,scale,norm=_normalize_for_fpga(h0,j0,n)
    meta={"original_h":[float(v) for v in hv],**norm}
    p=Problem(name,"ising",n,h,j,pad(jv,n),meta,scale);p.validate();return p
def parse_ising_text(text,n,name="ising"):
    d=json.loads(text);return make_ising(d["h"],d["J"],n,name)
def _round_shift_signed(value:int,shift:int)->int:
    if shift<=0:return int(value)
    bias=1<<(shift-1)
    return (value+bias)>>shift if value>=0 else -(((-value)+bias)>>shift)

def _quantized_payload_arrays(p:Problem):
    p.validate()
    hq=np.rint(np.asarray(p.h,dtype=np.float64)*float(SCALE)).astype(np.int64)
    ja=np.asarray(p.j,dtype=np.float64)
    jq=np.rint(ja[_UPPER_I,_UPPER_J]*float(SCALE)).astype(np.int64)
    if (hq.size and (hq.min()<Q_MIN or hq.max()>Q_MAX)) or (jq.size and (jq.min()<Q_MIN or jq.max()>Q_MAX)):
        raise ValueError("系数超出Q9.22范围")
    return hq,jq

def _round_shift_array(raw:np.ndarray,shift:int)->np.ndarray:
    if shift<=0:return raw.copy()
    bias=1<<(shift-1)
    return np.where(raw>=0,(raw+bias)>>shift,-(((-raw)+bias)>>shift))

def _try_bfp_payload(p:Problem,hq:np.ndarray,raw:np.ndarray,bits:int):
    limit=(1<<(bits-1))-1
    shift=max(0,int(np.max(np.abs(raw),initial=0)).bit_length()-(bits-1))
    encoded=_round_shift_array(raw,shift)
    while shift<31 and encoded.size and np.max(np.abs(encoded))>limit:
        shift+=1; encoded=_round_shift_array(raw,shift)
    if encoded.size and (np.any((raw!=0)&(encoded==0)) or np.max(np.abs(encoded))>limit): return None
    restored=encoded.astype(np.int64)<<shift
    if raw.size and float(np.max(np.abs(raw-restored)))/SCALE>1.0e-5:return None
    fmt=FORMAT_BFP8_J if bits==8 else FORMAT_BFP16_J
    flags=FLAG_UPPER_TRIANGLE|((shift&0x1F)<<1)
    header=bytes((encode_size_code(N_HW),encode_size_code(p.n),fmt,flags))
    hb=hq.astype('<i4',copy=False).tobytes()
    jb=encoded.astype(np.int8 if bits==8 else '<i2',copy=False).tobytes()
    return header+hb+jb

def _build_bfp8_j_payload(p:Problem):
    hq,raw=_quantized_payload_arrays(p); return _try_bfp_payload(p,hq,raw,8)

def _build_bfp16_j_payload(p:Problem):
    hq,raw=_quantized_payload_arrays(p); return _try_bfp_payload(p,hq,raw,16)

def build_load_payload(p:Problem)->bytes:
    cached=getattr(p,"_load_payload_cache",None)
    if cached is not None:return cached
    hq,raw=_quantized_payload_arrays(p)
    payload=_try_bfp_payload(p,hq,raw,8)
    if payload is None: payload=_try_bfp_payload(p,hq,raw,16)
    if payload is None:
        header=bytes((encode_size_code(N_HW),encode_size_code(p.n),FORMAT_Q9_22,FLAG_UPPER_TRIANGLE))
        payload=header+hq.astype('<i4',copy=False).tobytes()+raw.astype('<i4',copy=False).tobytes()
        assert len(payload)==LOAD_PAYLOAD_SIZE_Q9
    p._load_payload_cache=payload
    return payload



def _active_quantized_arrays(p: Problem):
    """Return Q9.22 h and upper-J for the active problem only."""
    cached=getattr(p, "_active_quantized_cache", None)
    if cached is not None:
        return cached
    p.validate()
    n=int(p.n)
    h=np.rint(np.asarray(p.h[:n],dtype=np.float64)*float(SCALE)).astype(np.int64)
    j=np.asarray(p.j,dtype=np.float64)[:n,:n]
    iu,ju=np.triu_indices(n,1)
    upper=np.rint(j[iu,ju]*float(SCALE)).astype(np.int64)
    if (h.size and (h.min()<Q_MIN or h.max()>Q_MAX)) or (upper.size and (upper.min()<Q_MIN or upper.max()>Q_MAX)):
        raise ValueError("活动区系数超出Q9.22范围")
    p._active_quantized_cache=(h,upper)
    return p._active_quantized_cache

def _try_uav_compact_payload(p: Problem, hq: np.ndarray, raw: np.ndarray, bits: int):
    limit=(1<<(bits-1))-1
    shift=max(0,int(np.max(np.abs(raw),initial=0)).bit_length()-(bits-1))
    encoded=_round_shift_array(raw,shift)
    while shift<31 and encoded.size and np.max(np.abs(encoded))>limit:
        shift+=1; encoded=_round_shift_array(raw,shift)
    if encoded.size and (np.any((raw!=0)&(encoded==0)) or np.max(np.abs(encoded))>limit):
        return None
    restored=encoded.astype(np.int64)<<shift
    # UAV BFP16 may tolerate one Q9.22 LSB-group of rounding while preserving
    # the low-energy ordering far better than BFP8. BFP8 keeps the strict guard.
    err = float(np.max(np.abs(raw-restored)))/SCALE if raw.size else 0.0
    if err > (1.0e-5 if bits==8 else 5.0e-5):
        return None
    fmt=FORMAT_UAV_BFP8_J if bits==8 else FORMAT_UAV_BFP16_J
    flags=FLAG_UPPER_TRIANGLE|((shift&0x1F)<<1)
    header=bytes((encode_size_code(N_HW),encode_size_code(p.n),fmt,flags))
    hb=hq.astype('<i4',copy=False).tobytes()
    # PL compact parser consumes complete 4-coefficient(BFP8) / 2-coefficient
    # (BFP16) words. Pad only the final packing group; padding is never used by
    # the active-N expansion.
    group=4 if bits==8 else 2
    pad=(-len(encoded))%group
    if pad:
        encoded=np.pad(encoded,(0,pad),constant_values=0)
    jb=encoded.astype(np.int8 if bits==8 else '<i2',copy=False).tobytes()
    return header+hb+jb

def _uav_sparse_bfp16_payload(p: Problem, dense_bfp16: bytes)->bytes:
    """Build wire-only sparse BFP16 upper-J records.

    Record layout is one little-endian u32: low16=active upper-triangle
    linear index, high16=signed BFP16 value. Firmware expands it back to the
    existing dense UAV-BFP16 stream before HP0 DMA, so PL RTL is unchanged.
    """
    n=int(p.n)
    upper_count=n*(n-1)//2
    j_offset=4+n*4
    padded=(upper_count+1)&~1
    dense=np.frombuffer(dense_bfp16[j_offset:],dtype='<i2',count=padded)[:upper_count]
    nz=np.flatnonzero(dense)
    if nz.size>0xFFFF:
        return dense_bfp16
    values_u16=dense[nz].astype(np.int16,copy=False).view(np.uint16).astype(np.uint32)
    records=(nz.astype(np.uint32) | (values_u16<<16)).astype('<u4',copy=False)
    header=bytes((encode_size_code(N_HW),encode_size_code(n),FORMAT_UAV_SPARSE_BFP16_J,dense_bfp16[3]))
    hb=dense_bfp16[4:j_offset]
    payload=header+hb+struct.pack('<HH',int(nz.size),0)+records.tobytes()
    return payload if len(payload)<len(dense_bfp16) else dense_bfp16

def _build_uav_compact_payload_python(p: Problem)->bytes:
    """Active-N FULL payload, normally BFP8; short UAV replans target one UDP datagram."""
    cached=getattr(p,'_uav_compact_payload_cache',None)
    if cached is not None:
        return cached
    hq,raw=_active_quantized_arrays(p)
    candidates=[]
    bfp8=_try_uav_compact_payload(p,hq,raw,8)
    if bfp8 is not None:
        candidates.append(bfp8)
    bfp16=_try_uav_compact_payload(p,hq,raw,16)
    if bfp16 is not None:
        candidates.append(bfp16)
        candidates.append(_uav_sparse_bfp16_payload(p,bfp16))
    payload=min(candidates,key=len) if candidates else None
    if payload is None:
        # Exact active-N Q9.22 fallback remains compact: even a 48-variable
        # problem is only ~4.7 KiB instead of the 131 KiB legacy padded frame.
        header=bytes((encode_size_code(N_HW),encode_size_code(p.n),FORMAT_UAV_Q9_22,FLAG_UPPER_TRIANGLE))
        payload=header+hq.astype('<i4',copy=False).tobytes()+raw.astype('<i4',copy=False).tobytes()
    p._uav_compact_payload_cache=payload
    return payload

def _build_uav_compact_h_payload_python(p: Problem)->bytes:
    """Active-N H-only payload used by UAV H-CACHE polishing."""
    cached=getattr(p,'_uav_compact_h_payload_cache',None)
    if cached is not None:
        return cached
    hq,_=_active_quantized_arrays(p)
    payload=bytes((encode_size_code(N_HW),encode_size_code(p.n),FORMAT_Q9_22,0))+hq.astype('<i4',copy=False).tobytes()
    p._uav_compact_h_payload_cache=payload
    return payload

def build_uav_compact_payload(p: Problem)->bytes:
    cached=getattr(p,'_uav_compact_payload_cache',None)
    if cached is not None:
        return cached
    native=None
    try:
        import native_fastpath as _nf
        native=_nf.uav_compact_payload_active(p.h,p.j,p.n,h_only=False)
    except Exception:
        native=None
    if native is not None:
        p._uav_compact_payload_cache=native
        p._native_wire_payload=True
        p._native_wire_payload_version=3001
        return native
    return _build_uav_compact_payload_python(p)


def build_uav_compact_h_payload(p: Problem)->bytes:
    cached=getattr(p,'_uav_compact_h_payload_cache',None)
    if cached is not None:
        return cached
    native=None
    try:
        import native_fastpath as _nf
        native=_nf.uav_compact_payload_active(p.h,p.j,p.n,h_only=True)
    except Exception:
        native=None
    if native is not None:
        p._uav_compact_h_payload_cache=native
        p._native_wire_payload=True
        p._native_wire_payload_version=3001
        return native
    return _build_uav_compact_h_payload_python(p)


def uav_compact_payload_info(payload: bytes):
    if len(payload)<4:
        return {'format':'invalid','bytes':len(payload),'active_n':0}
    fmt=payload[2]
    return {
        'format':'UAV-BFP8' if fmt==FORMAT_UAV_BFP8_J else ('UAV-BFP16' if fmt==FORMAT_UAV_BFP16_J else ('UAV-SPARSE-BFP16' if fmt==FORMAT_UAV_SPARSE_BFP16_J else ('UAV-Q9.22' if fmt==FORMAT_UAV_Q9_22 else 'unknown'))),
        'shift':(payload[3]>>1)&0x1F,
        'bytes':len(payload),
        'active_n':decode_size_code(payload[1]),
    }

def uav_compact_payload_fingerprints(payload: bytes):
    """Return full-problem and reusable-J fingerprints for Active-N payloads.

    The J/cache key must include the compact header.  Different active_n values
    can otherwise produce identical empty/sparse J byte regions, causing the
    host to send CMD_UAV_H_START while the board still has another active_n.
    """
    if len(payload)<4:
        raise ValueError("UAV compact payload too short")
    active_n=decode_size_code(payload[1])
    h_end=4+active_n*4
    if len(payload)<h_end:
        raise ValueError("UAV compact payload h section is incomplete")
    j_config=payload[:4]+payload[h_end:]
    return (binascii.crc32(payload)&0xFFFFFFFF,
            binascii.crc32(j_config)&0xFFFFFFFF)


def uav_logical_problem_fingerprints(p: Problem):
    """Conservative cache keys from the exact active-N Q9.22 logical problem.

    The payload fast path computes these keys *before* building any BFP/sparse wire
    payload.  Equality is therefore sufficient to guarantee that the encoded
    FPGA problem is identical, while a representationally-equivalent BFP
    payload may conservatively miss the cache.  That tradeoff never risks
    reusing stale FPGA state and lets UAV-CACHE skip all full-payload work.
    """
    cached=getattr(p,'_uav_logical_fingerprint_cache',None)
    if cached is not None:
        return cached
    hq,raw=_active_quantized_arrays(p)
    n=int(p.n)
    # Include active_n and a format-domain tag so 24- and 32-variable logical
    # problems can never alias even when J is sparse/empty.  CRC32 is only an
    # in-process cache discriminator; it is not sent to or trusted by FPGA.
    tag=struct.pack('<4sH',b'Q922',n)
    h_bytes=np.ascontiguousarray(hq,dtype='<i8').tobytes()
    j_bytes=np.ascontiguousarray(raw,dtype='<i8').tobytes()
    j_crc=binascii.crc32(tag)
    j_crc=binascii.crc32(j_bytes,j_crc)&0xFFFFFFFF
    full_crc=binascii.crc32(tag)
    full_crc=binascii.crc32(h_bytes,full_crc)
    full_crc=binascii.crc32(j_bytes,full_crc)&0xFFFFFFFF
    p._uav_logical_fingerprint_cache=(full_crc,j_crc)
    return p._uav_logical_fingerprint_cache

def load_payload_info(payload:bytes):
    if len(payload)<4:return {"format":"invalid","shift":0,"bytes":len(payload)}
    if payload[2]==FORMAT_BFP8_J:
        return {"format":"BFP8-J","shift":(payload[3]>>1)&0x1F,"bytes":len(payload)}
    if payload[2]==FORMAT_BFP16_J:
        return {"format":"BFP16-J","shift":(payload[3]>>1)&0x1F,"bytes":len(payload)}
    return {"format":"Q9.22-32","shift":0,"bytes":len(payload)}
def build_h_only_payload(p:Problem)->bytes:
    cached=getattr(p,"_h_payload_cache",None)
    if cached is not None:return cached
    p.validate()
    hq=np.rint(np.asarray(p.h,dtype=np.float64)*float(SCALE)).astype(np.int64)
    if hq.size and (hq.min()<Q_MIN or hq.max()>Q_MAX): raise ValueError("h系数超出Q9.22范围")
    payload=bytes((encode_size_code(N_HW),encode_size_code(p.n),FORMAT_Q9_22,0))+hq.astype('<i4',copy=False).tobytes()
    assert len(payload)==4+N_HW*4
    p._h_payload_cache=payload
    return payload
def payload_fingerprints(p:Problem,payload:Optional[bytes]=None):
    """Return CRC32 fingerprints over an already-quantized payload when supplied."""
    if payload is None:payload=build_load_payload(p)
    h_end=4+N_HW*4
    return (binascii.crc32(payload)&0xFFFFFFFF,
            binascii.crc32(payload[h_end:])&0xFFFFFFFF)

def decode_spins(raw:bytes,n:int=N_HW):
    if len(raw)<64:raise ValueError("256自旋结果需要64字节")
    value=int.from_bytes(raw[:64],"little");return [1 if ((value>>(2*i))&3)==1 else -1 if ((value>>(2*i))&3)==3 else 0 for i in range(n)]
def ising_energy(p,spins,original=False):
    """Fast active-N Ising energy used in the result path.

    The older full-matrix path scanned the padded 256x256 J matrix in Python for every FPGA
    response.  For UAV subproblems with only ~40-80 active variables this cost
    about 4-6 ms although all padded coefficients are zero.  Cache an active-N
    NumPy view and evaluate the identical expression with BLAS/vectorized code.
    """
    n=max(0,min(int(getattr(p,"n",N_HW)),len(spins),N_HW))
    if n<=0:return 0.0
    h_np=getattr(p,"_energy_h_np",None);j_np=getattr(p,"_energy_j_np",None)
    if h_np is None or j_np is None or len(h_np)!=n:
        h_np=np.asarray(p.h[:n],dtype=np.float64)
        j_np=np.asarray([row[:n] for row in p.j[:n]],dtype=np.float64)
        p._energy_h_np=h_np;p._energy_j_np=j_np
    s=np.asarray(spins[:n],dtype=np.float64)
    value=-float(np.dot(h_np,s))-float(s @ j_np @ s)
    return value/p.fpga_scale if original and p.fpga_scale else value
def maxcut_value(p,spins):return sum(p.source_matrix[i][j] for i in range(p.n) for j in range(i+1,p.n) if spins[i]!=spins[j]) if p.kind=="maxcut" else None
def qubo_value(p,spins):
    if p.kind!="qubo":return None
    n=max(0,min(int(p.n),len(spins)))
    if n<=0:return 0.0
    q_np=getattr(p,"_qubo_q_np",None)
    if q_np is None or q_np.shape!=(n,n):
        src=p.source_matrix
        if isinstance(src,np.ndarray):
            q_np=np.asarray(src[:n,:n],dtype=np.float64)
        else:
            q_np=np.asarray([row[:n] for row in src[:n]],dtype=np.float64)
        p._qubo_q_np=q_np
    x=np.fromiter((1.0 if spins[i]>0 else 0.0 for i in range(n)),dtype=np.float64,count=n)
    return float(x @ q_np @ x)
def kaiwu_qubo_matrix(p:Problem):
    n=p.n;q=[[0.0]*n for _ in range(n)]
    if p.kind=="qubo":
        for i in range(n):
            q[i][i]=p.source_matrix[i][i]
            for j in range(i+1,n):q[i][j]=p.source_matrix[i][j]+p.source_matrix[j][i]
    else:
        # Compare against the original, unscaled Ising objective.
        inv=1.0/p.fpga_scale if p.fpga_scale else 1.0
        for i in range(n):
            q[i][i]=-2*p.h[i]*inv+4*sum(p.j[i][j]*inv for j in range(n) if j!=i)
            for j in range(i+1,n):q[i][j]=-8*p.j[i][j]*inv
    return q
def export_solidification_bundle(path,p):
    chunks=(N_HW+31)//32
    def chunk(row,c):
        words=[quantize_q9_22(row[c*32+k])&0xFFFFFFFF if c*32+k<N_HW else 0 for k in range(32)]
        return "".join(f"{value:08X}" for value in reversed(words))
    h="\n".join(f"{quantize_q9_22(v)&0xFFFFFFFF:08X}" for v in p.h)+"\n"; jm="\n".join(chunk(p.j[i],c) for i in range(N_HW) for c in range(chunks))+"\n"
    with zipfile.ZipFile(path,"w",zipfile.ZIP_DEFLATED) as z:
        serializable={k:v for k,v in p.__dict__.items() if not k.startswith("_")}
        compact_payload=build_uav_compact_payload(p)
        compact_h_payload=build_uav_compact_h_payload(p)
        dense_payload=build_load_payload(p)
        compact_problem_crc,compact_j_crc=uav_compact_payload_fingerprints(compact_payload)
        payload_info={
            "online_transport":"Active-N compact",
            "load_start_command":"CMD_UAV_LOAD_START (0x30)",
            "h_start_command":"CMD_UAV_H_START (0x31)",
            "runtime_load_payload":"runtime_load_payload.bin",
            "runtime_h_payload":"runtime_h_payload.bin",
            "legacy_dense_payload":"legacy_dense_load_payload.bin",
            "compact_problem_crc32":f"0x{compact_problem_crc:08X}",
            "compact_j_config_crc32":f"0x{compact_j_crc:08X}",
            "compact":uav_compact_payload_info(compact_payload),
            "legacy_dense":load_payload_info(dense_payload),
        }
        z.writestr("runtime_load_payload.bin",compact_payload)
        z.writestr("runtime_h_payload.bin",compact_h_payload)
        z.writestr("runtime_payload_info.json",json.dumps(payload_info,ensure_ascii=False,indent=2))
        z.writestr("legacy_dense_load_payload.bin",dense_payload)
        z.writestr("default_h.mif",h)
        z.writestr("default_J.mif",jm)
        z.writestr("problem.json",json.dumps(serializable,ensure_ascii=False,indent=2))
