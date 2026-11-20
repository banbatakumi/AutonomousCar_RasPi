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
 * `touched` は h×w の 0 の作業領域（使った後に 0 へ戻す）。
 * 戻り値: 0 = 済、-1 = 作業領域を確保できなかった（`misses` は触らず `touched` も戻してある。呼び出し側は numpy 版で数える） */
int carve_touch(int64_t nr, double ox, double oy, const double *dx, const double *dy,
                 const double *d, int64_t ns, const float *t, double back,
                 double o0, double o1, double res, int64_t h, int64_t w,
                 uint16_t *misses, uint16_t count_max, uint8_t *touched)
{
    int64_t cap = 1024, nt = 0;
    int64_t *list = malloc(sizeof(int64_t) * (size_t)cap);
    if (list == NULL) return -1;
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
                    int64_t *grown = realloc(list, sizeof(int64_t) * (size_t)(cap * 2));
                    if (grown == NULL) {
                        touched[idx] = 0;
                        for (int64_t j = 0; j < nt; j++) touched[list[j]] = 0;
                        free(list);
                        return -1;
                    }
                    list = grown;
                    cap *= 2;
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
    return 0;
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
 * numpy 版は「候補 × 点」の配列を作ってから最後の軸で足す。np ≤ 8192。
 * 戻り値: 0 = 済、-1 = 作業領域を確保できなかった（呼び出し側は numpy 版で計算する） */
int sm_score_rows(const smap_t *m, int64_t nrow, const double *bx, const double *by,
                   int64_t np_, const double *rx, const double *ry, double inv2s2, double *out)
{
    double *like = malloc(sizeof(double) * (size_t)(np_ > 0 ? np_ : 1));
    if (like == NULL) return -1;
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
    return 0;
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

/* ── 全域の自己位置探索: 多解像度の分枝限定法（`slam2d/core/globalmatch.py`） ──
 *
 * Cartographer の FastCorrelativeScanMatcher と同じ考え方。尤度の格子 L（壁からの距離の exp(-d²/2σ²)）を
 * 2^h 四方の窓の最大値で縮めた格子 P_h を用意すると、2^h 四方の平行移動の候補をまとめた「節」の得点の
 * 上界は Σ_k P_h[点 k のセル] になる。上界が今の k 番目の候補より低い節は子を見ない。**全域の最良を厳密に
 * 見つける**（粗い総当たりのように刻みの間で真の姿勢を落とさない）。
 *
 * 格子の並び: 段 h は (H+pad)×(W+pad)、pad = 2^h − 1。セル (r, c)（r は −pad..H−1）の値は
 * buf[off[h] + (r + pad)·(W + pad) + (c + pad)]。P_0 = L。free も同じ並び（その窓に車が居られるセルがあるか）。
 * off_pts は角度ごとの点のセルのずれ (dx, dy)（[nang][npts][2]、int32）。
 * 候補は「場所（cells）と向き」が sep_cells・sep_ang 以上離れたものを上位 k 個まで返す
 * （out: [k][4] = 得点の和, 列, 行, 角度の番号）。戻り値は返した数。visited に調べた節の数。
 * 角度は [a0, a1) の番号だけ調べる。out の先頭 n_init 個は前の呼び出しの候補（続きから探す。
 * 角度を分けて複数周期で探しても、1 回で探したのと同じ上位になる）。
 */
typedef struct { float bound; int32_t ang, h, cx, cy; } bb_node;

typedef struct {
    int64_t H, W, nlev, npts, nang;
    const float *pyr; const int64_t *off; const uint8_t *free_; const int32_t *pts;
    int use_hint; double hx, hy, hr;
    double min_sum; int64_t k; double sep_cells, sep_ang; const double *angles;
    double *out; int64_t n_out; int64_t visited; int64_t max_visit;
} bb_ctx;

static inline float bb_bound(const bb_ctx *c, int32_t ang, int32_t h, int32_t cx, int32_t cy) {
    const int64_t pad = ((int64_t)1 << h) - 1, ww = c->W + pad;
    const float *g = c->pyr + c->off[h];
    const int32_t *p = c->pts + (int64_t)ang * c->npts * 2;
    float s = 0.0f;
    for (int64_t k = 0; k < c->npts; k++) {
        int64_t col = (int64_t)cx + p[2 * k], row = (int64_t)cy + p[2 * k + 1];
        if (col < -pad || row < -pad || col >= c->W || row >= c->H) continue;
        s += g[(row + pad) * ww + (col + pad)];
    }
    return s;
}

static inline int bb_free(const bb_ctx *c, int32_t h, int32_t cx, int32_t cy) {
    const int64_t pad = ((int64_t)1 << h) - 1, ww = c->W + pad;
    return c->free_[c->off[h] + (cy + pad) * ww + (cx + pad)] != 0;
}

/* 節（2^h 四方の候補の塊）がヒントの円に掛かるか */
static inline int bb_in_hint(const bb_ctx *c, int32_t h, int32_t cx, int32_t cy) {
    if (!c->use_hint) return 1;
    double sz = (double)((int64_t)1 << h);
    double nx = c->hx < cx ? cx : (c->hx > cx + sz - 1 ? cx + sz - 1 : c->hx);
    double ny = c->hy < cy ? cy : (c->hy > cy + sz - 1 ? cy + sz - 1 : c->hy);
    return (nx - c->hx) * (nx - c->hx) + (ny - c->hy) * (ny - c->hy) <= c->hr * c->hr;
}

static double bb_threshold(const bb_ctx *c) {
    if (c->n_out < c->k) return c->min_sum;
    double lo = c->out[0];
    for (int64_t i = 1; i < c->n_out; i++) if (c->out[4 * i] < lo) lo = c->out[4 * i];
    return lo > c->min_sum ? lo : c->min_sum;
}

static void bb_insert(bb_ctx *c, double s, int32_t cx, int32_t cy, int32_t ang) {
    double a = c->angles[ang];
    for (int64_t i = 0; i < c->n_out; i++) {
        double *o = c->out + 4 * i;
        double dx = o[1] - cx, dy = o[2] - cy;
        double da = fabs(fmod(c->angles[(int64_t)o[3]] - a + 3.0 * M_PI, 2.0 * M_PI) - M_PI);
        if (dx * dx + dy * dy < c->sep_cells * c->sep_cells && da < c->sep_ang) {
            if (s > o[0]) { o[0] = s; o[1] = cx; o[2] = cy; o[3] = ang; }
            return;
        }
    }
    int64_t slot;
    if (c->n_out < c->k) slot = c->n_out++;
    else {
        slot = 0;
        for (int64_t i = 1; i < c->n_out; i++) if (c->out[4 * i] < c->out[4 * slot]) slot = i;
        if (s <= c->out[4 * slot]) return;
    }
    double *o = c->out + 4 * slot;
    o[0] = s; o[1] = cx; o[2] = cy; o[3] = ang;
}

static void bb_descend(bb_ctx *c, int32_t ang, int32_t h, int32_t cx, int32_t cy, float bound) {
    if (c->max_visit > 0 && c->visited >= c->max_visit) return;
    if (bound <= bb_threshold(c)) return;
    if (h == 0) {
        bb_insert(c, bound, cx, cy, ang);
        return;
    }
    int32_t hh = h - 1, step = 1 << hh;
    bb_node ch[4];
    int n = 0;
    for (int a = 0; a < 2; a++) {
        for (int b = 0; b < 2; b++) {
            int32_t x = cx + a * step, y = cy + b * step;
            if (x >= c->W || y >= c->H) continue;
            if (!bb_free(c, hh, x, y) || !bb_in_hint(c, hh, x, y)) continue;
            c->visited++;
            ch[n].bound = bb_bound(c, ang, hh, x, y);
            ch[n].ang = ang; ch[n].h = hh; ch[n].cx = x; ch[n].cy = y;
            n++;
        }
    }
    /* 上界の高い子から（早く良い候補が見つかるほど枝刈りが効く） */
    for (int i = 1; i < n; i++) {
        bb_node t = ch[i]; int j = i - 1;
        while (j >= 0 && ch[j].bound < t.bound) { ch[j + 1] = ch[j]; j--; }
        ch[j + 1] = t;
    }
    for (int i = 0; i < n; i++) bb_descend(c, ch[i].ang, ch[i].h, ch[i].cx, ch[i].cy, ch[i].bound);
}

static int bb_cmp(const void *a, const void *b) {
    float x = ((const bb_node *)a)->bound, y = ((const bb_node *)b)->bound;
    return (x < y) - (x > y);
}

int64_t bb_match(int64_t H, int64_t W, int64_t nlev, const float *pyr, const int64_t *off,
                 const uint8_t *free_, int64_t nang, const double *angles, int64_t npts,
                 const int32_t *pts, int use_hint, double hx, double hy, double hr,
                 double min_sum, int64_t k, double sep_cells, double sep_ang,
                 int64_t max_visit, int64_t a0, int64_t a1, int64_t n_init,
                 double *out, int64_t *visited) {
    bb_ctx c = {H, W, nlev, npts, nang, pyr, off, free_, pts, use_hint, hx, hy, hr,
                min_sum, k, sep_cells, sep_ang, angles, out, n_init < k ? n_init : k, 0, max_visit};
    if (a0 < 0) a0 = 0;
    if (a1 > nang) a1 = nang;
    int32_t top = (int32_t)(nlev - 1), step = 1 << top;
    int64_t nx = (W + step - 1) / step, ny = (H + step - 1) / step;
    if (a1 <= a0) { *visited = 0; return c.n_out; }
    bb_node *roots = (bb_node *)malloc(sizeof(bb_node) * (size_t)((a1 - a0) * nx * ny));
    if (!roots) return -1;
    int64_t n = 0;
    for (int64_t a = a0; a < a1; a++)
        for (int64_t j = 0; j < ny; j++)
            for (int64_t i = 0; i < nx; i++) {
                int32_t x = (int32_t)(i * step), y = (int32_t)(j * step);
                if (!bb_free(&c, top, x, y) || !bb_in_hint(&c, top, x, y)) continue;
                roots[n].bound = bb_bound(&c, (int32_t)a, top, x, y);
                roots[n].ang = (int32_t)a; roots[n].h = top; roots[n].cx = x; roots[n].cy = y;
                n++;
            }
    c.visited = n;
    qsort(roots, (size_t)n, sizeof(bb_node), bb_cmp);
    for (int64_t i = 0; i < n; i++) {
        if (roots[i].bound <= bb_threshold(&c)) break;     /* 並べてあるので、以降は全部これ以下 */
        bb_descend(&c, roots[i].ang, roots[i].h, roots[i].cx, roots[i].cy, roots[i].bound);
    }
    free(roots);
    *visited = c.visited;
    return c.n_out;
}
