/*
 * ステレオの後処理（視差 → base_link の点 → 障害物の擬似 Scan）を C で行う。
 *
 * `raspi/core/stereo_match.py` の `obstacles_from_disparity`（numpy 版）と同じ結果を返す。式・丸め・
 * 境界の扱いを揃えてあり、`raspi/tests/test_stereo_native.py` が突き合わせる。**どちらかを直したら両方直す。**
 * Pi 5 では numpy 版が 1 組 約 12ms（点 3.4・変換 2.8・抽出 6.5ms）かかっていた。
 *
 * ビルドは `raspi/core/stereo_native.py` が初回の import 時に行う（cc -O2 -shared -fPIC）。
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    double r;
    double z;
} rz_t;

/*
 * a[0..n) の中で k 番目（0 始まり）に小さい値（並べ替えたときの a[k]）。a は並べ替わる。
 * 選ぶ値は並べ替えと同じなので numpy 版（np.sort してから添字）と結果が一致する。qsort より速い（O(n)）
 */
static double select_kth(double *a, long n, long k) {
    long lo = 0, hi = n - 1;
    while (lo < hi) {
        double pivot = a[lo + (hi - lo) / 2];
        long i = lo, j = hi;
        while (i <= j) {
            while (a[i] < pivot) i++;
            while (a[j] > pivot) j--;
            if (i <= j) {
                double tmp = a[i]; a[i] = a[j]; a[j] = tmp;
                i++; j--;
            }
        }
        if (k <= j) hi = j;
        else if (k >= i) lo = i;
        else return a[k];
    }
    return a[k];
}

/* 並べ替え済みの n 個の pct パーセント点（numpy 版 `_q` と同じ: 添字 round(pct/100*(n-1))、偶数丸め） */
static long q_index(double pct, long n) {
    return (long)nearbyint(pct / 100.0 * (double)(n - 1));
}

/*
 * 戻り値: 障害物とみなした点の数（高さ・距離の条件を満たした点）。
 * disp: h×w の float32（行の間隔 stride 要素）。無効は ≤ 0。
 * R, t: 平行化座標 → base_link（P_base = R·P + t、R は行優先 9 要素）。
 * work: 2·h·w 個の double の作業領域（呼び出し側が用意する）。
 * dist_out: 360 個。n_valid_out: 左端 skip_cols 列を除いた有効画素の数。
 */
long stereo_obstacles(const float *disp, long h, long w, long stride, long skip_cols,
                      double f, double baseline, double cx, double cy, double v0, double scale,
                      const double *R, const double *t,
                      double z_min, double z_max, double r_min, double r_max,
                      long min_points, double near_pct, double min_span, double slice_m,
                      double *work, double *dist_out, long *n_valid_out) {
    long n = 0, n_valid = 0;
    double *rs = work, *zs = work + h * w;
    int *degs = (int *)malloc(sizeof(int) * (size_t)(h * w));
    if (degs == NULL) return -1;
    for (int i = 0; i < 360; i++) dist_out[i] = 0.0;

    /* 割り算は 1 画素 1 回（Z）にし、行で決まる量は行ごとに出す。numpy 版とは丸めが最下位の桁で違うだけ（テストは 1e-9 で突き合わせる） */
    const double inv_f = 1.0 / f, fb = f * baseline, inv_s = 1.0 / scale;
    const double r_min2 = r_min * r_min, r_max2 = r_max * r_max;
    for (long v = 0; v < h; v++) {
        const float *row = disp + v * stride;
        double vv = ((double)v + 0.5) * inv_s - 0.5 + v0;
        double yf = (vv - cy) * inv_f;                 /* Y = yf·Z */
        for (long u = 0; u < w; u++) {
            float dv = row[u];
            if (!(dv > 0.0f)) continue;
            if (u >= skip_cols) n_valid++;
            double d = (double)dv * inv_s;
            double uu = ((double)u + 0.5) * inv_s - 0.5;
            double Z = fb / d;
            double X = (uu - d - cx) * inv_f * Z;
            double Y = yf * Z;
            double bz = R[6] * X + R[7] * Y + R[8] * Z + t[2];
            if (!isfinite(bz) || bz < z_min || bz > z_max) continue;
            double bx = R[0] * X + R[1] * Y + R[2] * Z + t[0];
            double by = R[3] * X + R[4] * Y + R[5] * Z + t[1];
            double r2 = bx * bx + by * by;
            if (r2 < r_min2 || r2 > r_max2) continue;
            double r = sqrt(r2);
            long deg = (long)nearbyint(atan2(by, bx) * (180.0 / M_PI));
            deg %= 360;
            if (deg < 0) deg += 360;
            rs[n] = r;
            zs[n] = bz;
            degs[n] = (int)deg;
            n++;
        }
    }
    *n_valid_out = n_valid;

    /* 方位ごとに分ける（数え上げで並べる） */
    long count[360] = {0}, start[361];
    for (long i = 0; i < n; i++) count[degs[i]]++;
    start[0] = 0;
    for (int k = 0; k < 360; k++) start[k + 1] = start[k] + count[k];
    size_t nn = (size_t)(n > 0 ? n : 1);
    rz_t *buf = (rz_t *)malloc(sizeof(rz_t) * nn);
    rz_t *cl = (rz_t *)malloc(sizeof(rz_t) * nn);      /* 刻みで並べ直した 1 方位ぶん */
    double *ztmp = (double *)malloc(sizeof(double) * nn);
    long *kbuf = (long *)malloc(sizeof(long) * nn);
    if (buf == NULL || cl == NULL || ztmp == NULL || kbuf == NULL) {
        free(degs); free(buf); free(cl); free(ztmp); free(kbuf);
        return -1;
    }
    long fill[360];
    memcpy(fill, start, sizeof(fill));
    for (long i = 0; i < n; i++) {
        long p = fill[degs[i]]++;
        buf[p].r = rs[i];
        buf[p].z = zs[i];
    }

    /*
     * 近い方から、距離 slice_m 刻みの「縦に広がったかたまり」を探す（numpy 版 `_nearest_cluster`）。
     * numpy 版は方位の点を全部 r で並べるが、ここでは刻み k = (r − 最小の r)/slice_m で数え上げで振り分け、
     * 調べるかたまり（刻み kk と kk+1）の中で要る順位の値だけを選ぶ（`select_kth`）。値の集合が同じなので結果も同じ（SGBM で点が増えて
     * 方位ごとの qsort が Pi で数 ms になったため）。
     */
    long kmax_all = (long)((r_max - r_min) / slice_m) + 2;
    long *kcnt = (long *)calloc((size_t)(kmax_all + 2), sizeof(long));
    long *kst = (long *)malloc(sizeof(long) * (size_t)(kmax_all + 3));
    if (kcnt == NULL || kst == NULL) {
        free(degs); free(buf); free(cl); free(ztmp); free(kbuf); free(kcnt); free(kst);
        return -1;
    }
    for (int k = 0; k < 360; k++) {
        long c = count[k];
        if (c < min_points) continue;
        rz_t *b = buf + start[k];
        double r0 = b[0].r;
        for (long i = 1; i < c; i++) if (b[i].r < r0) r0 = b[i].r;
        long km = 0;
        for (long i = 0; i < c; i++) {
            long kk = (long)((b[i].r - r0) / slice_m);
            if (kk > kmax_all) kk = kmax_all;          /* r は r_min〜r_max なので起きない（念のため） */
            kbuf[i] = kk;
            if (kk > km) km = kk;
        }
        memset(kcnt, 0, sizeof(long) * (size_t)(km + 2));
        for (long i = 0; i < c; i++) kcnt[kbuf[i]]++;
        kst[0] = 0;
        for (long j = 0; j <= km; j++) kst[j + 1] = kst[j] + kcnt[j];
        memcpy(kcnt, kst, sizeof(long) * (size_t)(km + 1));    /* 書き込み位置 */
        for (long i = 0; i < c; i++) cl[kcnt[kbuf[i]]++] = b[i];
        for (long kk = 0; kk <= km; kk++) {
            if (kst[kk + 1] == kst[kk]) continue;      /* 点の無い刻み（numpy 版の np.unique に無い） */
            long lo = kst[kk], hi = kst[kk + 2 <= km + 1 ? kk + 2 : km + 1];
            long m = hi - lo;
            if (m < min_points) continue;
            for (long j = 0; j < m; j++) ztmp[j] = cl[lo + j].z;
            double z90 = select_kth(ztmp, m, q_index(90.0, m));
            double z10 = select_kth(ztmp, m, q_index(10.0, m));
            if (z90 - z10 >= min_span) {
                for (long j = 0; j < m; j++) ztmp[j] = cl[lo + j].r;
                dist_out[k] = select_kth(ztmp, m, q_index(near_pct, m));
                break;
            }
        }
    }
    free(kcnt);
    free(kst);
    free(cl);
    free(degs);
    free(buf);
    free(ztmp);
    free(kbuf);
    return n;
}
