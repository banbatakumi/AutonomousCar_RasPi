/* レーシングラインの計算の重い内側ループ（`raspi/nav/native.py` が ctypes で読む）。
 *
 * ★ Python 版（`raspi/nav/raceline.py`）と**同じ順の演算**で書いてあり、結果はビット一致する
 *   （`raspi/tests/test_native.py`）。`-ffp-contract=off`（積和の融合を禁止）で組むこと。
 *   式を変えるときは Python 版と一緒に変え、テストで一致を確かめる。
 *
 * 帯行列の解法（`band_solve`）だけは Python 版（密行列の `np.linalg.solve`）と丸めが違う
 * （同じ連立方程式を別の手順で解く）。
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/* ── 速度プロファイル（`speed_profile` のスカラー版のループ） ── */

/* 速度 vj（曲率 kj）の点から距離 d だけ加速（減速）したときに届く速度（`speed_profile.reach`） */
static inline double reach(double vj, double kj, double a_x, double d, double inv)
{
    if (inv != 0.0) {
        double ay = vj * vj * kj;
        double r = 1.0 - ay * ay * inv;
        a_x = r > 0.0 ? a_x * sqrt(r) : 0.0;
    }
    return sqrt(vj * vj + 2.0 * a_x * d);
}

/* 隣の点までの距離 ds[i] = |xy[i+1] − xy[i]|（閉じていれば最後は始点まで）。`ds` は n 個 */
void seg_len(int64_t n, const double *xy, int closed, double *ds)
{
    for (int64_t i = 0; i + 1 < n; i++)
        ds[i] = hypot(xy[2 * i + 2] - xy[2 * i], xy[2 * i + 3] - xy[2 * i + 1]);
    if (closed && n > 0)
        ds[n - 1] = hypot(xy[0] - xy[2 * n - 2], xy[1] - xy[2 * n - 1]);
}

/* 閉じた経路: 後ろ向き → 前向きを2周（`vl` を書き換える） */
void sp_closed(int64_t n, double *vl, const double *kl, const double *ds,
               double a_brake, double a_accel, double inv)
{
    for (int rep = 0; rep < 2; rep++) {
        for (int64_t i = n - 1; i >= 0; i--) {
            int64_t j = i + 1 < n ? i + 1 : 0;
            double lim = reach(vl[j], kl[j], a_brake, ds[i], inv);
            if (lim < vl[i]) vl[i] = lim;
        }
        for (int64_t i = 0; i < n; i++) {
            int64_t j = i > 0 ? i - 1 : n - 1;
            double lim = reach(vl[j], kl[j], a_accel, ds[j], inv);
            if (lim < vl[i]) vl[i] = lim;
        }
    }
}

/* 開いた経路の後ろ向き（終点から） */
void sp_open_back(int64_t n, double *vl, const double *kl, const double *ds,
                  double a_brake, double inv)
{
    for (int64_t i = n - 2; i >= 0; i--) {
        double lim = reach(vl[i + 1], kl[i + 1], a_brake, ds[i], inv);
        if (lim < vl[i]) vl[i] = lim;
    }
}

/* 開いた経路の前向き（始点から） */
void sp_open_fwd(int64_t n, double *vl, const double *kl, const double *ds,
                 double a_accel, double inv)
{
    for (int64_t i = 1; i < n; i++) {
        double lim = reach(vl[i - 1], kl[i - 1], a_accel, ds[i - 1], inv);
        if (lim < vl[i]) vl[i] = lim;
    }
}

/* ── 曲率（`curvature`・`chord_curvature`） ── */

/* 3点 a, b, c の外接円の 2·cross/den（`den` が小さければ 0）。符号つき */
static inline double circ(const double *a, const double *b, const double *c)
{
    double ab = hypot(b[0] - a[0], b[1] - a[1]);
    double bc = hypot(c[0] - b[0], c[1] - b[1]);
    double ca = hypot(a[0] - c[0], a[1] - c[1]);
    double cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    double den = ab * bc * ca;
    return den > 1e-9 ? 2.0 * cross / (den > 1e-9 ? den : 1e-9) : 0.0;
}

/* `curvature`（左旋回が正）。`xy` は (n, 2) の行優先 */
void curvature(int64_t n, const double *xy, int closed, double *out)
{
    for (int64_t i = 0; i < n; i++) {
        int64_t a = i > 0 ? i - 1 : n - 1, c = i + 1 < n ? i + 1 : 0;
        out[i] = circ(xy + 2 * a, xy + 2 * i, xy + 2 * c);
    }
    if (!closed && n >= 3) {
        out[0] = out[1];
        out[n - 1] = out[n - 2];
    }
}

/* `chord_curvature`（絶対値）。`k` 点離れた3点 */
void chord_curvature(int64_t n, const double *xy, int64_t k, int closed, double *out)
{
    for (int64_t i = 0; i < n; i++) {
        int64_t a, c;
        if (closed) {
            a = ((i - k) % n + n) % n;
            c = (i + k) % n;
        } else {
            a = i - k < 0 ? 0 : i - k;
            c = i + k > n - 1 ? n - 1 : i + k;
        }
        out[i] = fabs(circ(xy + 2 * a, xy + 2 * i, xy + 2 * c));
    }
}

/* `a[0..m)` の小さい方から `k` 番目（0 始まり）の値。`a` は並べ替わる（Hoare の選択、O(m)） */
static double select_kth(double *a, int64_t m, int64_t k)
{
    int64_t lo = 0, hi = m - 1;
    while (lo < hi) {
        double piv = a[lo + (hi - lo) / 2];
        int64_t i = lo, j = hi;
        while (i <= j) {
            while (a[i] < piv) i++;
            while (a[j] > piv) j--;
            if (i <= j) {
                double t = a[i]; a[i] = a[j]; a[j] = t;
                i++; j--;
            }
        }
        if (k <= j) hi = j;
        else if (k >= i) lo = i;
        else return a[k];
    }
    return a[k];
}

/* `speed_kappa`（n ≥ 3）。`chord` [m] の弦の曲率。点の間隔は隣どうしの距離の中央値 */
void speed_kappa(int64_t n, const double *xy, int closed, double chord, double *out)
{
    double *seg = malloc(sizeof(double) * (size_t)(n - 1));
    for (int64_t i = 0; i < n - 1; i++)
        seg[i] = hypot(xy[2 * i + 2] - xy[2 * i], xy[2 * i + 3] - xy[2 * i + 1]);
    int64_t m = n - 1;
    /* `np.median`: 偶数個なら中央の2つの平均（和 ÷ 2）。小さい方は、上の半分を選んだ後の
     * 下の部分（a[0..m/2)）の最大 */
    double step;
    if (m % 2) {
        step = select_kth(seg, m, m / 2);
    } else {
        double hi_v = select_kth(seg, m, m / 2), lo_v = seg[0];
        for (int64_t i = 1; i < m / 2; i++)
            if (seg[i] > lo_v) lo_v = seg[i];
        step = (lo_v + hi_v) / 2.0;
    }
    free(seg);
    /* Python の `round` は偶数への丸め。`nearbyint`（既定の丸めモード）と同じ */
    double kk = nearbyint(chord / 2.0 / (step > 1e-6 ? step : 1e-6));
    int64_t k = kk > 1.0 ? (int64_t)kk : 1;
    int64_t kmax = (n - 1) / 2 > 1 ? (n - 1) / 2 : 1;
    if (k > kmax) k = kmax;
    chord_curvature(n, xy, k, closed, out);
    if (!closed) {
        for (int64_t i = 0; i < k; i++) out[i] = out[k];
        for (int64_t i = n - k; i < n; i++) out[i] = out[n - k - 1];
    }
}

/* ── 車体外形の余裕（`_BodyCheck.clearance`） ── */

/* `floor(t / res)` と同じ値（`inv` = 1/res）。割り算は遅い（Pi で1回の評価の大半）ので逆数を掛け、
 * 整数の境界の 1e-9 以内に来たときだけ割り算でやり直す（掛け算の誤差は |t/res|·2⁻⁵² 程度で、
 * 境界から離れていれば floor の値は変わらない） */
static inline double floor_div(double t, double res, double inv)
{
    double q = t * inv, f = floor(q);
    if (q - f < 1e-9 || f + 1.0 - q < 1e-9) return floor(t / res);
    return f;
}

static void body_core(int64_t n, const double *xy, const double *off, int closed,
                      const double *tn, const uint8_t *todo, int64_t k, const double *cx,
                      const double *cy, const float *edt, int64_t h, int64_t w,
                      double ox0, double oy0, double res, float *out);

/* 各点の、`k` 個の円のうち壁にいちばん近いものの距離（`edt` の値、float32）を `out` へ。
 * 呼び出し側が半径を引く（numpy の型の昇格を Python 側に残すため）。
 * `off` は NULL か (n, 2)（位置だけずらす） */
void body_min_edt(int64_t n, const double *xy, const double *off, int closed,
                  int64_t k, const double *cx, const double *cy,
                  const float *edt, int64_t h, int64_t w,
                  double ox0, double oy0, double res, float *out)
{
    body_core(n, xy, off, closed, NULL, NULL, k, cx, cy, edt, h, w, ox0, oy0, res, out);
}

/* `tn` は NULL か、閉じた経路の d2[i] = |xy[i+2] − xy[i]|（点 i の接線の長さは d2[i−1]。
 * `refine_kappa` が作ったものを使い回す。同じ被演算子の hypot なので値は同じ）。
 * `todo` は NULL（全点）か、計算する点の印 */
static void body_core(int64_t n, const double *xy, const double *off, int closed,
                      const double *tn, const uint8_t *todo, int64_t k, const double *cx,
                      const double *cy, const float *edt, int64_t h, int64_t w,
                      double ox0, double oy0, double res, float *out)
{
    double inv = 1.0 / res;
    for (int64_t i = 0; i < n; i++) {
        if (todo && !todo[i]) continue;
        double dx, dy, nn;
        if (closed) {
            int64_t a = i + 1 < n ? i + 1 : 0, b = i > 0 ? i - 1 : n - 1;
            dx = xy[2 * a] - xy[2 * b];
            dy = xy[2 * a + 1] - xy[2 * b + 1];
            nn = tn ? tn[b] : hypot(dx, dy);
        } else {
            if (n < 2) {
                dx = dy = 0.0;
            } else if (i == 0) {
                dx = (xy[2] - xy[0]) / 1.0;
                dy = (xy[3] - xy[1]) / 1.0;
            } else if (i == n - 1) {
                dx = (xy[2 * i] - xy[2 * i - 2]) / 1.0;
                dy = (xy[2 * i + 1] - xy[2 * i - 1]) / 1.0;
            } else {
                dx = (xy[2 * i + 2] - xy[2 * i - 2]) / 2.0;
                dy = (xy[2 * i + 3] - xy[2 * i - 1]) / 2.0;
            }
            nn = hypot(dx, dy);
        }
        if (nn < 1e-9) nn = 1.0;
        double c = dx / nn, s = dy / nn;
        double px0 = xy[2 * i], py0 = xy[2 * i + 1];
        if (off) {
            px0 = px0 + off[2 * i];
            py0 = py0 + off[2 * i + 1];
        }
        float best = INFINITY;
        for (int64_t j = 0; j < k; j++) {
            double px = px0 + cx[j] * c - cy[j] * s;
            double py = py0 + cx[j] * s + cy[j] * c;
            double fc = floor_div(px - ox0, res, inv), fr = floor_div(py - oy0, res, inv);
            float v;
            if (fc >= 0.0 && fc < (double)w && fr >= 0.0 && fr < (double)h)
                v = edt[(int64_t)fr * w + (int64_t)fc];
            else
                v = 0.0f;
            if (v < best || j == 0) best = v;
        }
        out[i] = best;
    }
}

/* ── 帯行列の連立方程式（`min_curvature_alpha` の自由な点の部分問題） ──
 *
 * 行列（対称・正定値、大きさ N）は、対角 d0[p]、1つ隣 d1[p] = S(p,p+1)、2つ隣 d2[p] = S(p,p+2)
 * と、巡回の角 c20 = S(N-2,0)・c10 = S(N-1,0)・c11 = S(N-1,1) だけが非零。末尾2行・2列を縁にした
 * 矢形として Cholesky 分解する（先頭 K=N-2 は帯幅2の帯、縁は密）。O(N)。N ≥ 6。
 * 戻り値: 0 = 解けた、1 = 正定値でない（呼び出し側が密行列で解き直す）。 */
int band_solve(int64_t N, const double *d0, const double *d1, const double *d2,
               double c20, double c10, double c11, const double *rhs, double *x)
{
    int64_t K = N - 2;
    double *l0 = malloc(sizeof(double) * (size_t)(5 * K));
    double *l1 = l0 + K, *l2 = l0 + 2 * K, *y0 = l0 + 3 * K, *y1 = l0 + 4 * K;
    int rc = 1;
    /* 先頭の帯: L[i][i] = l0, L[i][i-1] = l1, L[i][i-2] = l2 */
    for (int64_t i = 0; i < K; i++) {
        double a2 = i >= 2 ? d2[i - 2] / l0[i - 2] : 0.0;
        double a1 = 0.0;
        if (i >= 1) a1 = (d1[i - 1] - (i >= 2 ? a2 * l1[i - 1] : 0.0)) / l0[i - 1];
        double p = d0[i] - a2 * a2 - a1 * a1;
        if (!(p > 0.0)) goto done;
        l2[i] = a2;
        l1[i] = a1;
        l0[i] = sqrt(p);
    }
    /* 縁の2行 B（N-2 行と N-1 行の先頭 K 列）を L で前進代入: y = L⁻¹ Bᵀ */
    for (int64_t i = 0; i < K; i++) {
        double b0 = 0.0, b1 = 0.0;
        if (i == 0) { b0 += c20; b1 += c10; }
        if (i == 1) b1 += c11;
        if (i == K - 2) b0 += d2[K - 2];               /* S(N-2, N-4) */
        if (i == K - 1) { b0 += d1[K - 1]; b1 += d2[K - 1]; }  /* S(N-2,N-3), S(N-1,N-3) */
        double s0 = b0, s1 = b1;
        if (i >= 1) { s0 -= l1[i] * y0[i - 1]; s1 -= l1[i] * y1[i - 1]; }
        if (i >= 2) { s0 -= l2[i] * y0[i - 2]; s1 -= l2[i] * y1[i - 2]; }
        y0[i] = s0 / l0[i];
        y1[i] = s1 / l0[i];
    }
    /* 2×2 のシューア補行列 D − y yᵀ を Cholesky */
    double s00 = d0[N - 2], s01 = d1[N - 2], s11 = d0[N - 1];
    for (int64_t i = 0; i < K; i++) {
        s00 -= y0[i] * y0[i];
        s01 -= y0[i] * y1[i];
        s11 -= y1[i] * y1[i];
    }
    if (!(s00 > 0.0)) goto done;
    double m00 = sqrt(s00), m10 = s01 / m00, q = s11 - m10 * m10;
    if (!(q > 0.0)) goto done;
    double m11 = sqrt(q);
    /* 前進: z = L⁻¹ r（先頭）、縁は r_b − y z */
    double *z = x;
    for (int64_t i = 0; i < K; i++) {
        double s = rhs[i];
        if (i >= 1) s -= l1[i] * z[i - 1];
        if (i >= 2) s -= l2[i] * z[i - 2];
        z[i] = s / l0[i];
    }
    double r0 = rhs[N - 2], r1 = rhs[N - 1];
    for (int64_t i = 0; i < K; i++) { r0 -= y0[i] * z[i]; r1 -= y1[i] * z[i]; }
    double w0 = r0 / m00, w1 = (r1 - m10 * w0) / m11;
    /* 後退: 縁 → 先頭 */
    double xb1 = w1 / m11, xb0 = (w0 - m10 * xb1) / m00;
    x[N - 2] = xb0;
    x[N - 1] = xb1;
    for (int64_t i = K - 1; i >= 0; i--) {
        double s = z[i] - y0[i] * xb0 - y1[i] * xb1;
        if (i + 1 < K) s -= l1[i + 1] * x[i + 1];
        if (i + 2 < K) s -= l2[i + 2] * x[i + 2];
        x[i] = s / l0[i];
    }
    rc = 0;
done:
    free(l0);
    return rc;
}

/* 点がすべて `mask`（h×w、0/1）の 1 のセルにあるか（`_Band.inside`）。外へ出たら 0 */
int all_inside(int64_t n, const double *xy, const uint8_t *mask, int64_t h, int64_t w,
               double ox0, double oy0, double res)
{
    double inv = 1.0 / res;
    for (int64_t i = 0; i < n; i++) {
        double fc = floor_div(xy[2 * i] - ox0, res, inv), fr = floor_div(xy[2 * i + 1] - oy0, res, inv);
        if (!(fc >= 0.0 && fc < (double)w && fr >= 0.0 && fr < (double)h)) return 0;
        if (!mask[(int64_t)fr * w + (int64_t)fc]) return 0;
    }
    return 1;
}

/* ── 仕上げの評価（`optimize` の `cost`）をまとめて回す ──
 * Python 版は numpy の小さな演算を1評価に数十回呼び、Pi ではその呼び出しの手間が計算そのものより重い。
 * 式は上の各関数（＝ Python 版）と同じ。 */

/* 閉じた経路の d[i] = |xy[i+k] − xy[i]|（添字は巡回）。`todo` が NULL でなければ印の点だけ */
static void dist_k(int64_t n, const double *xy, int64_t k, const uint8_t *todo, double *d)
{
    for (int64_t i = 0; i < n; i++) {
        if (todo && !todo[i]) continue;
        int64_t j = i + k < n ? i + k : i + k - n;      /* 0 ≤ k < n（剰余は Pi で遅い） */
        d[i] = hypot(xy[2 * j] - xy[2 * i], xy[2 * j + 1] - xy[2 * i + 1]);
    }
}

/* 3点 a, b, c（辺の長さ ab, bc, ca は与える）の 2·cross/den。`circ` と同じ値 */
static inline double circ_d(const double *a, const double *b, const double *c,
                            double ab, double bc, double ca)
{
    double cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    double den = ab * bc * ca;
    return den > 1e-9 ? 2.0 * cross / (den > 1e-9 ? den : 1e-9) : 0.0;
}

/* 点 i の値が、点 i+o（o ∈ offs、巡回）のどれかが変わったら計算し直しになる、という印を `todo` へ */
static void mark(int64_t n, const uint8_t *ch, const int64_t *offs, int no, uint8_t *todo)
{
    for (int64_t i = 0; i < n; i++) todo[i] = 0;
    for (int j = 0; j < no; j++) {
        int64_t o = offs[j];                    /* −n < o < n */
        for (int64_t i = 0; i < n; i++) {
            int64_t q = i + o;
            q = q < 0 ? q + n : q >= n ? q - n : q;
            todo[i] |= ch[q];
        }
    }
}

/* 前回の点列 `base`（2n）と比べて変わった点の印（`valid` が 0 なら全部）。`base` を `xy` に更新する */
static int64_t diff_points(int64_t n, const double *xy, double *base, int valid, uint8_t *ch)
{
    int64_t c = 0;
    for (int64_t i = 0; i < n; i++) {
        uint8_t d = !valid || xy[2 * i] != base[2 * i] || xy[2 * i + 1] != base[2 * i + 1];
        ch[i] = d;
        c += d;
        if (d) {
            base[2 * i] = xy[2 * i];
            base[2 * i + 1] = xy[2 * i + 1];
        }
    }
    return c;
}

/* 仕上げの評価（`optimize` の `cost`）の前半: 速度を決める曲率 ks（閉じた経路、`speed_kappa`）と
 * `stats` = (ks の最大, 3点の曲率の絶対値の最大)。
 *
 * ★ 1回の評価で変わるのはこぶの周りの点だけ（ビット単位で同じ点が平均 3/4、magp2）なので、点ごとの
 *   値は前回の評価（`kc` に持つ）から使い回し、入力の点が変わった所だけ計算し直す。同じ入力からは
 *   同じ値が出るので、結果は全部を計算し直したときと同じ。
 * ★ 辺の長さは同じ2点の組の hypot を1回だけ計算して使い回す（ab_i = d_k[i−k]、bc_i = d_k[i]、
 *   ca_i = d_2k[i−k]。a − c は c − a の符号反転で、hypot は符号によらない）。
 *
 * `kc` は 9n 個: [base 2n | d1 | d2 | dk | d2k | ks | cu]（d1[i] = |xy[i+1]−xy[i]|、d2[i] = |xy[i+2]−xy[i]|、
 * cu は3点の曲率の絶対値）。`meta` = [有効か, k]。点の数が変わったら呼び出し側が `meta[0]` を 0 にする */
void refine_kappa(int64_t n, const double *xy, double chord, double *kc, int64_t *meta,
                  double *stats)
{
    double *base = kc, *d1 = kc + 2 * n, *d2 = kc + 3 * n, *dk = kc + 4 * n, *d2k = kc + 5 * n,
           *ks = kc + 6 * n, *cu = kc + 7 * n;
    uint8_t *ch = malloc((size_t)(2 * n)), *todo = ch + n;
    double *seg = malloc(sizeof(double) * (size_t)n);
    int valid = (int)meta[0];
    diff_points(n, xy, base, valid, ch);
    const int64_t o01[] = {0, 1}, o02[] = {0, 2}, o101[] = {-1, 0, 1};
    mark(n, ch, o01, 2, todo);
    dist_k(n, xy, 1, todo, d1);
    mark(n, ch, o02, 2, todo);
    dist_k(n, xy, 2, todo, d2);
    /* 点の間隔 = 隣どうしの距離（巡回しない n−1 個）の中央値（`speed_kappa`） */
    int64_t m = n - 1;
    for (int64_t i = 0; i < m; i++) seg[i] = d1[i];
    double step;
    if (m % 2) {
        step = select_kth(seg, m, m / 2);
    } else {
        double hi_v = select_kth(seg, m, m / 2), lo_v = seg[0];
        for (int64_t i = 1; i < m / 2; i++)
            if (seg[i] > lo_v) lo_v = seg[i];
        step = (lo_v + hi_v) / 2.0;
    }
    double kk = nearbyint(chord / 2.0 / (step > 1e-6 ? step : 1e-6));
    int64_t k = kk > 1.0 ? (int64_t)kk : 1;
    int64_t kmax = (n - 1) / 2 > 1 ? (n - 1) / 2 : 1;
    if (k > kmax) k = kmax;
    int full = !valid || k != meta[1];
    /* 弦の辺の長さ（k が前回と違えば全部） */
    const int64_t o0k[] = {0, k}, o02k[] = {0, 2 * k}, okk[] = {-k, 0, k};
    mark(n, ch, o0k, 2, todo);
    dist_k(n, xy, k, full ? NULL : todo, dk);
    mark(n, ch, o02k, 2, todo);
    dist_k(n, xy, 2 * k, full ? NULL : todo, d2k);
    mark(n, ch, okk, 3, todo);
    for (int64_t i = 0; i < n; i++) {
        if (!full && !todo[i]) continue;
        int64_t a = i - k < 0 ? i - k + n : i - k, b = i + k < n ? i + k : i + k - n;
        ks[i] = fabs(circ_d(xy + 2 * a, xy + 2 * i, xy + 2 * b, dk[a], dk[i], d2k[a]));
    }
    mark(n, ch, o101, 3, todo);
    for (int64_t i = 0; i < n; i++) {
        if (!todo[i]) continue;
        int64_t p = i > 0 ? i - 1 : n - 1, q = i + 1 < n ? i + 1 : 0;
        cu[i] = fabs(circ_d(xy + 2 * p, xy + 2 * i, xy + 2 * q, d1[p], d1[i], d2[p]));
    }
    double mx = ks[0], c = cu[0];
    for (int64_t i = 1; i < n; i++) {
        if (ks[i] > mx) mx = ks[i];
        if (cu[i] > c) c = cu[i];
    }
    meta[0] = 1;
    meta[1] = k;
    stats[0] = mx;
    stats[1] = c;
    free(seg);
    free(ch);
}

/* numpy の `np.add.reduce`（float64・連続・8192 個以下）と同じ手順の和（pairwise） */
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

/* 後半: 帯の内外（`_Band.inside`）→ 車体の余裕（`_BodyCheck.clearance` の最小 − r が `clr_ok` 未満なら落とす）
 * → 速度プロファイル（閉・スカラー）→ 走行時間（`_travel_time`）。
 * 戻り値: 1 = 帯の外、2 = 車体の余裕が足りない、0 = `t_out` に時間。n ≤ 8192。
 * `kc` は同じ点列で呼んだ `refine_kappa` の作業領域（ks と距離）。車体の余裕も点ごとに前回の値を
 * 使い回す（`bc` は 3n 個: [base 2n | 余裕 n（float32 を double の領域に置く）]、`bmeta` = [有効か]。
 * 点 i の余裕は点 i−1, i, i+1 で決まる）。
 * `mask`・`edt` は NULL なら検査しない。`r32` は円の半径を float32 にしたもの（numpy の型の昇格と同じ） */
int refine_eval(int64_t n, const double *xy, const double *kc, double *bc, int64_t *bmeta,
                const uint8_t *mask, int64_t mh, int64_t mw, double mox, double moy, double mres,
                const float *edt, int64_t eh, int64_t ew, double eox, double eoy, double eres,
                int64_t k, const double *cx, const double *cy, float r32, double clr_ok,
                double v_max, double v_min, double a_lat, double a_accel, double a_brake,
                double inv, double *t_out)
{
    if (mask && !all_inside(n, xy, mask, mh, mw, mox, moy, mres)) return 1;
    const double *d1 = kc + 2 * n, *d2 = kc + 3 * n, *ks = kc + 6 * n;
    if (edt) {
        float *clr = (float *)(bc + 2 * n);
        uint8_t *ch = malloc((size_t)(2 * n)), *todo = ch + n;
        diff_points(n, xy, bc, (int)bmeta[0], ch);
        const int64_t o101[] = {-1, 0, 1};
        mark(n, ch, o101, 3, todo);
        body_core(n, xy, NULL, 1, d2, todo, k, cx, cy, edt, eh, ew, eox, eoy, eres, clr);
        free(ch);
        bmeta[0] = 1;
        float lo = clr[0];
        for (int64_t i = 1; i < n; i++)
            if (clr[i] < lo) lo = clr[i];
        float d = lo - r32;
        if ((double)d < clr_ok) return 2;
    }
    double *buf = malloc(sizeof(double) * (size_t)(3 * n));
    double *vl = buf, *fl = buf + n, *term = buf + 2 * n;
    for (int64_t i = 0; i < n; i++) {
        double kk = ks[i] > 1e-6 ? ks[i] : 1e-6;
        double s = sqrt(a_lat / kk);
        double v = v_max <= s ? v_max : s;
        vl[i] = v;
        fl[i] = v <= v_min ? v : v_min;
    }
    sp_closed(n, vl, ks, d1, a_brake, a_accel, inv);
    for (int64_t i = 0; i < n; i++) {
        double v = vl[i] >= fl[i] ? vl[i] : fl[i];
        vl[i] = v >= 1e-3 ? v : 1e-3;
    }
    for (int64_t i = 0; i < n; i++)
        term[i] = 2.0 * d1[i] / (vl[i] + vl[i + 1 < n ? i + 1 : 0]);
    *t_out = pairwise_sum(term, n);
    free(buf);
    return 0;
}

/* ── 道路グラフの細線化（`roadgraph._thin`、Zhang-Suen） ──
 * `img` は外周1画素を 0 で埋めた (h+2)×(w+2) の 0/1。Python 版と同じく、各段（step 0/1）で消す画素を
 * 段の初めの画像から全部決めてから消す（並列の更新）。整数の演算だけなので結果は同じ */
void zs_thin(int64_t h, int64_t w, uint8_t *img, int64_t max_iter)
{
    int64_t W = w + 2;
    uint8_t *rm = malloc((size_t)((h + 2) * W));
    for (int64_t it = 0; it < max_iter; it++) {
        int changed = 0;
        for (int step = 0; step < 2; step++) {
            int any = 0;
            for (int64_t r = 1; r <= h; r++) {
                for (int64_t c = 1; c <= w; c++) {
                    int64_t i = r * W + c;
                    rm[i] = 0;
                    if (!img[i]) continue;
                    /* P2..P9（上から時計回り） */
                    int p[8] = {img[i - W], img[i - W + 1], img[i + 1], img[i + W + 1],
                                img[i + W], img[i + W - 1], img[i - 1], img[i - W - 1]};
                    int b = 0, a = 0;
                    for (int k = 0; k < 8; k++) {
                        b += p[k];
                        a += p[k] == 0 && p[(k + 1) & 7] == 1;
                    }
                    int c1, c2;
                    if (step == 0) {
                        c1 = p[0] * p[2] * p[4] == 0;
                        c2 = p[2] * p[4] * p[6] == 0;
                    } else {
                        c1 = p[0] * p[2] * p[6] == 0;
                        c2 = p[0] * p[4] * p[6] == 0;
                    }
                    if (b >= 2 && b <= 6 && a == 1 && c1 && c2) {
                        rm[i] = 1;
                        any = 1;
                    }
                }
            }
            if (any) {
                for (int64_t r = 1; r <= h; r++)
                    for (int64_t c = 1; c <= w; c++)
                        if (rm[r * W + c]) img[r * W + c] = 0;
                changed = 1;
            }
        }
        if (!changed) break;
    }
    free(rm);
}
