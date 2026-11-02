/* 占有格子のレイの行進（`slam2d/core/grid.py` の `OccGrid.raycast`・`_carve`）。`core/native.py` が ctypes で読む。
 *
 * ★ Python 版（numpy）と**同じ型・同じ順の演算**で書いてあり、結果はビット一致する
 *   （`slam2d/tests/test_native.py`）。`-ffp-contract=off` で組むこと（`core/cbuild.py`）。
 *   numpy 版は全部のレイを最後の歩まで一時配列で進めるが、こちらは当たった所で止める。
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/* `raycast` の、各レイの最初に壁（か格子の外）に当たった歩の番号（0 始まり。当たらなければ −1）。
 *
 * numpy 版の式（float32）:
 *   x = x0 + cos * t[k]、col = floor((x − o0) / res) を int32 に（y・row も同じ）、
 *   格子の外は壁扱い、中なら grid[row, col]。
 * `x0`・`y0` は `n0` 個（1 なら全レイで共通）。`o0`・`o1`・`res` は float32 にしたもの */
void ray_first_hit(int64_t nr, const float *x0, const float *y0, int64_t n0,
                   const float *c, const float *s, int64_t ns, const float *t,
                   const uint8_t *grid, int64_t h, int64_t w,
                   float o0, float o1, float res, int64_t *first)
{
    for (int64_t i = 0; i < nr; i++) {
        float xo = x0[n0 == 1 ? 0 : i], yo = y0[n0 == 1 ? 0 : i];
        float ci = c[i], si = s[i];
        int64_t hit = -1;
        for (int64_t k = 0; k < ns; k++) {
            float px = xo + ci * t[k];
            float py = yo + si * t[k];
            float fc = floorf((px - o0) / res), fr = floorf((py - o1) / res);
            /* numpy は int32 にしてから 0 ≤ col < w を見る。床関数の値（整数）のまま比べても同じ */
            if (!(fc >= 0.0f && fc < (float)w && fr >= 0.0f && fr < (float)h)
                || grid[(int64_t)fr * w + (int64_t)fc]) {
                hit = k;
                break;
            }
        }
        first[i] = hit;
    }
}

/* `_carve`: レイの手前のセル（終端の `back` [m] 手前まで）を「素通り」として1周に1回だけ数える
 * （`_bump`: 触ったセルを +1、`count_max` で頭打ち）。
 *
 * numpy 版の式（float64。`t` は float32 を float64 にしたもの）:
 *   x = ox + dx * t[k]、col = floor((x − o0) / res) を int32 に、格子の中で t[k] < d − back の歩を数える。
 * `touched` は h×w の 0 の作業領域（使った後に 0 へ戻す） */
void carve_touch(int64_t nr, double ox, double oy, const double *dx, const double *dy,
                 const double *d, int64_t ns, const float *t, double back,
                 double o0, double o1, double res, int64_t h, int64_t w,
                 uint16_t *misses, uint16_t count_max, uint8_t *touched)
{
    int64_t cap = 1024, nt = 0;
    int64_t *list = malloc(sizeof(int64_t) * (size_t)cap);
    for (int64_t i = 0; i < nr; i++) {
        double lim = d[i] - back;
        for (int64_t k = 0; k < ns; k++) {
            double tk = (double)t[k];
            if (!(tk < lim)) break;                 /* t は増えていくので、ここから先は全部外れ */
            double px = ox + dx[i] * tk, py = oy + dy[i] * tk;
            double fc = floor((px - o0) / res), fr = floor((py - o1) / res);
            if (!(fc >= 0.0 && fc < (double)w && fr >= 0.0 && fr < (double)h)) continue;
            int64_t idx = (int64_t)fr * w + (int64_t)fc;
            if (!touched[idx]) {
                touched[idx] = 1;
                if (nt == cap) {
                    cap *= 2;
                    list = realloc(list, sizeof(int64_t) * (size_t)cap);
                }
                list[nt++] = idx;
            }
        }
    }
    for (int64_t j = 0; j < nt; j++) {
        int64_t idx = list[j];
        if (misses[idx] < count_max) misses[idx] += 1;
        touched[idx] = 0;
    }
    free(list);
}

/* ── 面要素地図の対応付け（`core/surfmap.py` の `SurfaceMap.associate`） ── */

/* `SurfaceMap` の配列（`core/native.py` の `SmapC` と同じ並び） */
typedef struct {
    int64_t h, w;
    double o0, o1, res;
    const int64_t *nearest;      /* (h, w) → 壁の index か −1 */
    const uint8_t *known;        /* (h, w) か NULL（全部既知） */
    const double *qx, *qy, *nx, *ny;
    const uint8_t *planar, *support;
    double gate, max_dist;
} smap_t;

/* 1点ぶんの `associate`（numpy 版と同じ式・同じ順。対応が無効な点も index 0 の面要素を返す） */
static inline void assoc1(const smap_t *m, double wx, double wy, double *oqx, double *oqy,
                          double *onx, double *ony, uint8_t *opl, uint8_t *ovalid, double *odist)
{
    double fc = floor((wx - m->o0) / m->res), fr = floor((wy - m->o1) / m->res);
    int ok = fc >= 0.0 && fc < (double)m->w && fr >= 0.0 && fr < (double)m->h;
    int64_t idx = ok ? (int64_t)fr * m->w + (int64_t)fc : 0;
    int64_t j = ok ? m->nearest[idx] : -1;
    if (m->known) ok = ok && m->known[idx];
    ok = ok && j >= 0;
    int64_t jj = ok ? j : 0;
    double qx = m->qx[jj], qy = m->qy[jj], nx = m->nx[jj], ny = m->ny[jj];
    uint8_t planar = m->planar[jj];
    ok = ok && m->support[jj];
    double ex = wx - qx, ey = wy - qy;
    double e = hypot(ex, ey);
    double d = planar ? fabs(nx * ex + ny * ey) : e;
    double tang = fabs(-ny * ex + nx * ey);
    int beyond = planar && tang > m->gate;
    int far = e > m->max_dist;
    int valid = ok && !far && !beyond && d < m->max_dist;
    *oqx = qx; *oqy = qy; *onx = nx; *ony = ny; *opl = planar;
    *ovalid = (uint8_t)valid;
    *odist = valid ? d : INFINITY;
}

void sm_associate(const smap_t *m, int64_t n, const double *wx, const double *wy,
                  double *qx, double *qy, double *nx, double *ny, uint8_t *planar,
                  uint8_t *valid, double *dist)
{
    for (int64_t i = 0; i < n; i++)
        assoc1(m, wx[i], wy[i], qx + i, qy + i, nx + i, ny + i, planar + i, valid + i, dist + i);
}

/* numpy の和（float64・連続・8192 個以下の 1 行）と同じ手順（pairwise） */
static double pairwise_sum(const double *a, int64_t n)
{
    if (n < 8) {
        double res = 0.0;
        for (int64_t i = 0; i < n; i++) res += a[i];
        return res;
    }
    if (n <= 128) {
        double r[8];
        int64_t i;
        for (int j = 0; j < 8; j++) r[j] = a[j];
        for (i = 8; i < n - (n % 8); i += 8)
            for (int j = 0; j < 8; j++) r[j] += a[i + j];
        double res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        for (; i < n; i++) res += a[i];
        return res;
    }
    int64_t n2 = n / 2;
    n2 -= n2 % 8;
    return pairwise_sum(a, n2) + pairwise_sum(a + n2, n - n2);
}

/* 総当たりの採点（`register.search_steps`・`localize.Localizer.step`）: 候補 r の点 p を
 * (bx[r] + rx[p], by[r] + ry[p]) に置き、`exp(−min(d, 10)² · inv2s2)`（対応が無効なら 0）の点の和を `out[r]` へ。
 * numpy 版は「候補 × 点」の配列を作ってから最後の軸で足す。np ≤ 8192 */
void sm_score_rows(const smap_t *m, int64_t nrow, const double *bx, const double *by,
                   int64_t np_, const double *rx, const double *ry, double inv2s2, double *out)
{
    double *like = malloc(sizeof(double) * (size_t)(np_ > 0 ? np_ : 1));
    for (int64_t r = 0; r < nrow; r++) {
        for (int64_t p = 0; p < np_; p++) {
            double qx, qy, nx, ny, d;
            uint8_t pl, valid;
            assoc1(m, bx[r] + rx[p], by[r] + ry[p], &qx, &qy, &nx, &ny, &pl, &valid, &d);
            double dm = d < 10.0 ? d : 10.0;
            like[p] = valid ? exp(-(dm * dm) * inv2s2) : 0.0;
        }
        out[r] = pairwise_sum(like, np_);
    }
    free(like);
}

/* ── 面要素の法線の材料（`SurfaceMap.__init__` の近傍の和） ──
 * 壁セル i（`rows[i]`・`cols[i]`）の周り (2r+1)² セルの壁セル j について、px = qx[j] − qx[i]（py も）の
 * 和・2乗の和・積の和と個数を足す。numpy 版は近傍のずれ（dr 外・dc 内）を外側、点を内側に回すので、
 * 点ごとに見ると同じずれの順に足している。ここでは点を外側にして同じ順に足す（結果は同じ） */
void normal_sums(int64_t n, const int64_t *rows, const int64_t *cols, const int64_t *idx_map,
                 int64_t h, int64_t w, const double *qx, const double *qy, int64_t r,
                 double *sx, double *sy, double *sxx, double *syy, double *sxy, double *cnt)
{
    for (int64_t i = 0; i < n; i++) {
        double ax = 0.0, ay = 0.0, axx = 0.0, ayy = 0.0, axy = 0.0, ac = 0.0;
        double xi = qx[i], yi = qy[i];
        for (int64_t dr = -r; dr <= r; dr++) {
            int64_t rr = rows[i] + dr;
            if (rr < 0 || rr >= h) continue;
            for (int64_t dc = -r; dc <= r; dc++) {
                int64_t cc = cols[i] + dc;
                if (cc < 0 || cc >= w) continue;
                int64_t j = idx_map[rr * w + cc];
                if (j < 0) continue;
                double px = qx[j] - xi, py = qy[j] - yi;
                ax += px; ay += py;
                axx += px * px; ayy += py * py; axy += px * py;
                ac += 1.0;
            }
        }
        sx[i] = ax; sy[i] = ay; sxx[i] = axx; syy[i] = ayy; sxy[i] = axy; cnt[i] = ac;
    }
}
