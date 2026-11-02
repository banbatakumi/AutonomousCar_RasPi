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

/* ── Reeds-Shepp の最短経路の長さ（`raspi/nav/reeds_shepp.py` の `shortest_path(...).length`） ──
 * Hybrid A* のヒューリスティックが子ノードごとに呼ぶ（Python 版は候補 48 通りを経路のオブジェクトで
 * 組み立てていて、探索時間の大半）。Python 版と同じ式・同じ順で、**長さだけ**を計算する。
 * ★ Python の `math.hypot` は libm ではなく CPython 独自の高精度の計算（`vector_norm`）、`%` は Python の剰余、
 *   `sum` は 3.12 以降の補償つきの和（Neumaier）。どれも CPython の実装を写してある（`test_native.py` で突き合わせる） */

#define RS_EPS 1e-9
static const double RS_PI = 3.141592653589793;

/* CPython の float の `%`（符号は割る数に合わせる） */
static inline double py_mod(double x, double y)
{
    double m = fmod(x, y);
    if (m != 0.0) {
        if ((y < 0) != (m < 0)) m += y;
    } else {
        m = copysign(0.0, y);
    }
    return m;
}

/* CPython の `math.hypot(x, y)`（Modules/mathmodule.c の vector_norm、2 引数） */
static double py_hypot(double x, double y)
{
    double v[2] = {fabs(x), fabs(y)};
    if (isnan(v[0]) || isnan(v[1])) {
        if (isinf(v[0]) || isinf(v[1])) return INFINITY;
        return NAN;
    }
    double mx = v[0] > v[1] ? v[0] : v[1];
    if (isinf(mx)) return mx;
    if (mx == 0.0) return mx;
    int max_e;
    frexp(mx, &max_e);
    if (max_e < -1023) {
        /* 非正規数の領域（ここでは来ない）。CPython と同じく DBL_MIN で割ってやり直す */
        return 2.2250738585072014e-308 * py_hypot(v[0] / 2.2250738585072014e-308,
                                                 v[1] / 2.2250738585072014e-308);
    }
    double scale = ldexp(1.0, -max_e);
    double csum = 1.0, frac1 = 0.0, frac2 = 0.0;
    for (int i = 0; i < 2; i++) {
        double t = v[i] * scale;
        double hi = t * t, lo = fma(t, t, -hi);           /* dl_mul */
        double sh = csum + hi, sl = (csum - sh) + hi;      /* dl_fast_sum */
        csum = sh;
        frac1 += lo;
        frac2 += sl;
    }
    double h = sqrt(csum - 1.0 + (frac1 + frac2));
    double hi = -h * h, lo = fma(-h, h, -hi);
    double sh = csum + hi, sl = (csum - sh) + hi;
    csum = sh;
    frac1 += lo;
    frac2 += sl;
    double t = csum - 1.0 + (frac1 + frac2);
    h += t / (2.0 * h);
    return h / scale;
}

static inline double rs_mod2pi(double th)
{
    double v = py_mod(th, 2.0 * RS_PI);
    if (v < 0.0) v += 2.0 * RS_PI;
    return v;
}

static inline double rs_wrap_pi(double th)
{
    return py_mod(th + RS_PI, 2.0 * RS_PI) - RS_PI;
}

static void rs_tau_omega(double u, double v, double xi, double eta, double phi, int signed_omega,
                         double *tau, double *omega)
{
    double delta = rs_mod2pi(u - v);
    double a = sin(u) - sin(delta);
    double b = cos(u) - cos(delta) - 1.0;
    double t1 = atan2(eta * a - xi * b, xi * a + eta * b);
    double t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;
    *tau = t2 < 0 ? rs_mod2pi(t1 + RS_PI) : rs_mod2pi(t1);
    double w = *tau - u + v - phi;
    *omega = signed_omega ? rs_wrap_pi(w) : rs_mod2pi(w);
}

/* 語族 k（`_GENERATORS` の順）を (x, y, phi) で解き、区間の符号つき長さを `w` へ。区間の数（解が無ければ 0） */
static int rs_word(int k, double x, double y, double phi, double *w)
{
    const double hp = 0.5 * RS_PI;
    switch (k) {
    case 0: {   /* _lsl */
        double px = x - sin(phi), py = y - 1.0 + cos(phi);
        double u = py_hypot(px, py), t = atan2(py, px);
        if (t >= -RS_EPS) {
            double v = rs_mod2pi(phi - t);
            if (v >= -RS_EPS) { w[0] = t; w[1] = u; w[2] = v; return 3; }
        }
        return 0;
    }
    case 1: {   /* _lsr */
        double px = x + sin(phi), py = y - 1.0 - cos(phi);
        double u1 = py_hypot(px, py), t1 = atan2(py, px);
        if (u1 * u1 >= 4.0) {
            double u = sqrt(u1 * u1 - 4.0);
            double t = rs_mod2pi(t1 + atan2(2.0, u));
            double v = rs_mod2pi(t - phi);
            if (t >= -RS_EPS && v >= -RS_EPS) { w[0] = t; w[1] = u; w[2] = v; return 3; }
        }
        return 0;
    }
    case 2:     /* _lrl_fwd */
    case 3: {   /* _lrl_back */
        double xi = x - sin(phi), eta = y - 1.0 + cos(phi);
        double u1 = py_hypot(xi, eta), theta = atan2(eta, xi);
        if (u1 <= 4.0) {
            double a = acos(0.25 * u1);
            double t = rs_mod2pi(theta + 0.5 * RS_PI + a);
            double u = rs_mod2pi(RS_PI - 2.0 * a);
            if (k == 2) {
                double v = rs_mod2pi(phi - t - u);
                if (t >= -RS_EPS && u >= -RS_EPS) { w[0] = t; w[1] = -u; w[2] = v; return 3; }
            } else {
                double v = rs_mod2pi(t + u - phi);
                if (t >= -RS_EPS && u >= -RS_EPS && v >= -RS_EPS) {
                    w[0] = t; w[1] = -u; w[2] = -v; return 3;
                }
            }
        }
        return 0;
    }
    case 4: {   /* _lrlr_a */
        double xi = x + sin(phi), eta = y - 1.0 - cos(phi);
        double rho = (2.0 + py_hypot(xi, eta)) / 4.0;
        if (rho <= 1.0) {
            double u = acos(rho), t, v;
            rs_tau_omega(u, -u, xi, eta, phi, 1, &t, &v);
            if (t >= -RS_EPS && v <= RS_EPS) { w[0] = t; w[1] = u; w[2] = -u; w[3] = v; return 4; }
        }
        return 0;
    }
    case 5: {   /* _lrlr_b */
        double xi = x + sin(phi), eta = y - 1.0 - cos(phi);
        double rho = (20.0 - xi * xi - eta * eta) / 16.0;
        if (0.0 <= rho && rho <= 1.0) {
            double u = -acos(rho);
            if (u >= -0.5 * RS_PI) {
                double t, v;
                rs_tau_omega(u, u, xi, eta, phi, 0, &t, &v);
                if (t >= -RS_EPS && v >= -RS_EPS) { w[0] = t; w[1] = u; w[2] = u; w[3] = v; return 4; }
            }
        }
        return 0;
    }
    case 6: {   /* _lrsl */
        double xi = x - sin(phi), eta = y - 1.0 + cos(phi);
        double rho = py_hypot(xi, eta), theta = atan2(eta, xi);
        if (rho >= 2.0) {
            double r = sqrt(rho * rho - 4.0);
            double u = 2.0 - r;
            double t = rs_mod2pi(theta + atan2(r, -2.0));
            double v = rs_wrap_pi(phi - 0.5 * RS_PI - t);
            if (t >= -RS_EPS && u <= RS_EPS && v <= RS_EPS) {
                w[0] = t; w[1] = -hp; w[2] = u; w[3] = v; return 4;
            }
        }
        return 0;
    }
    case 7: {   /* _lrsr */
        double xi = x + sin(phi), eta = y - 1.0 - cos(phi);
        double rho = py_hypot(-eta, xi), theta = atan2(xi, -eta);
        if (rho >= 2.0) {
            double t = theta, u = 2.0 - rho;
            double v = rs_wrap_pi(t + 0.5 * RS_PI - phi);
            if (t >= -RS_EPS && u <= RS_EPS && v <= RS_EPS) {
                w[0] = t; w[1] = -hp; w[2] = u; w[3] = v; return 4;
            }
        }
        return 0;
    }
    case 8: {   /* _lrslr */
        double xi = x + sin(phi), eta = y - 1.0 - cos(phi);
        double rho = py_hypot(xi, eta);
        if (rho >= 2.0) {
            double u = 4.0 - sqrt(rho * rho - 4.0);
            if (u <= RS_EPS) {
                double t = rs_mod2pi(atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
                double v = rs_mod2pi(t - phi);
                if (t >= -RS_EPS && v >= -RS_EPS) {
                    w[0] = t; w[1] = -hp; w[2] = u; w[3] = -hp; w[4] = v; return 5;
                }
            }
        }
        return 0;
    }
    }
    return 0;
}

/* `sum(s.length for s in segments)`（CPython 3.12 以降の浮動小数の和: Neumaier の補償つき） */
static double py_sum(const double *a, int n)
{
    double f = 0.0, c = 0.0;
    for (int i = 0; i < n; i++) {
        double x = a[i], t = f + x;
        if (fabs(f) >= fabs(x)) c += (f - t) + x;
        else c += (x - t) + f;
        f = t;
    }
    if (c != 0.0 && isfinite(c)) f += c;
    return f;
}

/* 正規化した (x, y, phi) の候補（`_candidates`）の長さの最小を `*best` に畳み込む。
 * `rev` は逆向きの問題の解（区間の順が逆になる。長さは区間の並べ替えで和の順が変わるので逆順に足す） */
/* テスト用: 候補の長さを全部書き出す先（NULL なら書かない） */
static double *rs_dump = NULL;
static int rs_ndump = 0;

static void rs_fold(double x, double y, double phi, double R, int rev, double *best, int *found)
{
    const double args[4][3] = {{x, y, phi}, {-x, y, -phi}, {x, -y, -phi}, {-x, -y, phi}};
    for (int k = 0; k < 9; k++) {
        for (int s = 0; s < 4; s++) {
            double w[5], seg[5];
            int n = rs_word(k, args[s][0], args[s][1], args[s][2], w);
            if (!n) continue;
            int negate = s == 1 || s == 3, m = 0;
            for (int i = 0; i < n; i++) {
                double len = negate ? -w[i] : w[i];
                if (fabs(len) > RS_EPS) seg[m++] = fabs(len) * 1.0;   /* `_build`（半径 1） */
            }
            if (!m) continue;
            double scaled[5];
            for (int i = 0; i < m; i++) scaled[i] = seg[rev ? m - 1 - i : i] * R;
            double L = py_sum(scaled, m);
            if (rs_dump && rs_ndump < 128) rs_dump[rs_ndump++] = L;
            if (!*found || L < *best) { *best = L; *found = 1; }
        }
    }
}

/* `reeds_shepp.shortest_path(start, goal, R).length` と同じ値 */
double rs_shortest_length(double sx, double sy, double syaw, double gx, double gy, double gyaw, double R)
{
    double dx = gx - sx, dy = gy - sy;
    if (py_hypot(dx, dy) < 1e-9 && fabs(rs_mod2pi(gyaw - syaw)) < 1e-9) return 0.0;
    double c = cos(syaw), s = sin(syaw);
    double lx = (c * dx + s * dy) / R, ly = (-s * dx + c * dy) / R;
    double lphi = rs_mod2pi(gyaw - syaw);
    double best = 0.0;
    int found = 0;
    rs_fold(lx, ly, lphi, R, 0, &best, &found);
    double cg = cos(gyaw), sg = sin(gyaw);
    rs_fold(-(cg * dx + sg * dy) / R, -(-sg * dx + cg * dy) / R, rs_mod2pi(syaw - gyaw), R, 1,
            &best, &found);
    return found ? best : 0.0;
}

/* 語族ごとの区間の文字（`_GENERATORS` の順。0 = L、1 = R、2 = S） */
static const int RS_LETTERS[9][5] = {
    {0, 2, 0}, {0, 2, 1}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0, 1}, {0, 1, 0, 1},
    {0, 1, 2, 0}, {0, 1, 2, 1}, {0, 1, 2, 0, 1}};

/* 正規化した (x, y, phi) の候補（`_candidates`）を書き出す。`rev` は逆向きの問題の解（区間の順と前後を反転）。
 * 区間は実スケール（`candidate_paths` の `out`）: 曲率 ±1/R か 0、長さ |len|·1·R */
static void rs_emit(double x, double y, double phi, double R, int rev, int8_t *gear, double *curv,
                    double *len, int64_t *nseg, double *total, int64_t *count, int64_t cap)
{
    const double args[4][3] = {{x, y, phi}, {-x, y, -phi}, {x, -y, -phi}, {-x, -y, phi}};
    for (int k = 0; k < 9; k++) {
        for (int s = 0; s < 4; s++) {
            double w[5];
            int n = rs_word(k, args[s][0], args[s][1], args[s][2], w);
            if (!n) continue;
            int negate = s == 1 || s == 3, swap = s >= 2, m = 0;
            int8_t g[5];
            double cv[5], ln[5];
            for (int i = 0; i < n; i++) {
                double l = negate ? -w[i] : w[i];
                if (!(fabs(l) > RS_EPS)) continue;
                int letter = RS_LETTERS[k][i];
                if (swap && letter < 2) letter = 1 - letter;
                double c1 = letter == 0 ? 1.0 / 1.0 : letter == 1 ? -1.0 / 1.0 : 0.0;
                g[m] = l > 0.0 ? 1 : -1;
                cv[m] = c1;
                ln[m] = fabs(l) * 1.0;
                m++;
            }
            if (!m || *count >= cap) continue;
            int64_t o = *count * 5;
            double scaled[5];
            for (int i = 0; i < m; i++) {
                int j = rev ? m - 1 - i : i;
                gear[o + i] = (int8_t)(rev ? -g[j] : g[j]);
                curv[o + i] = cv[j] / R;
                len[o + i] = ln[j] * R;
                scaled[i] = len[o + i];
            }
            nseg[*count] = m;
            total[*count] = py_sum(scaled, m);
            (*count)++;
        }
    }
}

/* `reeds_shepp.candidate_paths` の候補を、**並べ替える前の順**（順向き → 逆向き）で書き出す。
 * 各候補は区間 `nseg[i]` 個（`gear`・`curv`・`len` の i*5 から）と長さ `total[i]`。
 * 戻り値は候補の数。既に目標姿勢なら −1（Python 版は空の経路 1 本を返す） */
int64_t rs_candidates(double sx, double sy, double syaw, double gx, double gy, double gyaw, double R,
                      int8_t *gear, double *curv, double *len, int64_t *nseg, double *total, int64_t cap)
{
    double dx = gx - sx, dy = gy - sy;
    if (py_hypot(dx, dy) < 1e-9 && fabs(rs_mod2pi(gyaw - syaw)) < 1e-9) return -1;
    double c = cos(syaw), s = sin(syaw);
    double lx = (c * dx + s * dy) / R, ly = (-s * dx + c * dy) / R;
    int64_t count = 0;
    rs_emit(lx, ly, rs_mod2pi(gyaw - syaw), R, 0, gear, curv, len, nseg, total, &count, cap);
    double cg = cos(gyaw), sg = sin(gyaw);
    rs_emit(-(cg * dx + sg * dy) / R, -(-sg * dx + cg * dy) / R, rs_mod2pi(syaw - gyaw), R, 1,
            gear, curv, len, nseg, total, &count, cap);
    return count;
}

/* テスト用: CPython の `math.hypot`・`%`・`sum` を写した関数をそのまま出す */
double rs_py_hypot(double x, double y) { return py_hypot(x, y); }
double rs_py_mod(double x, double y) { return py_mod(x, y); }
double rs_py_sum(const double *a, int64_t n) { return py_sum(a, (int)n); }

/* テスト用: 候補の長さを全部（Python の `candidate_paths` の並べ替えの前の順に）`out` へ。個数を返す */
int64_t rs_all_lengths(double sx, double sy, double syaw, double gx, double gy, double gyaw, double R,
                       double *out)
{
    rs_dump = out;
    rs_ndump = 0;
    rs_shortest_length(sx, sy, syaw, gx, gy, gyaw, R);
    rs_dump = NULL;
    return rs_ndump;
}

/* ── Hybrid A* の子ノード（`raspi/nav/hybrid_astar.py` の `_successors` の前半） ──
 * ギア（前・後）× 曲率ごとに、弧を `ns` 等分して進めた姿勢（`_advance`）に車体の円被覆を置き、
 * 壁だけ・壁＋未知の距離場の値から余裕を出す（`LocalMap.body_clearance_both`）。Python 版と同じ式・同じ型
 * （距離場は float32、余裕は float32 − float32(半径)）。出力は子 i ごとに端点の姿勢・余裕の最小・端点の余裕 */

static inline double hy_wrap(double a) { return py_mod(a + RS_PI, 2.0 * RS_PI) - RS_PI; }

void hy_successors(double x0, double y0, double yaw0, int64_t nc, const double *curvs, double step,
                   int64_t ns, const float *wall_edt, const float *blk_edt, int64_t h, int64_t w,
                   double o0, double o1, double res, int64_t k, const double *cx, const double *cy,
                   float r32, double *pose, float *wall_min, float *unk_min, float *wall_end)
{
    int64_t child = 0;
    for (int gi = 0; gi < 2; gi++) {
        int gear = gi == 0 ? 1 : -1;
        for (int64_t ci = 0; ci < nc; ci++, child++) {
            double curv = curvs[ci];
            double ds = step * gear;
            double x = x0, y = y0, yaw = yaw0;
            float wmin = 0.0f, umin = 0.0f, wlast = 0.0f;
            for (int64_t i = 0; i < ns; i++) {
                double d = ds / (double)ns;
                if (fabs(curv) < 1e-9) {
                    x = x + d * cos(yaw);
                    y = y + d * sin(yaw);
                } else {
                    double dyaw = d * curv, r = 1.0 / curv;
                    double nx = x + r * (sin(yaw + dyaw) - sin(yaw));
                    double ny = y - r * (cos(yaw + dyaw) - cos(yaw));
                    x = nx;
                    y = ny;
                    yaw = hy_wrap(yaw + dyaw);
                }
                double c = cos(yaw), s = sin(yaw);
                float vw = 0.0f, vu = 0.0f;
                for (int64_t j = 0; j < k; j++) {
                    double px = x + cx[j] * c - cy[j] * s;
                    double py = y + cx[j] * s + cy[j] * c;
                    double fc = floor((px - o0) / res), fr = floor((py - o1) / res);
                    float a = 0.0f, b = 0.0f;
                    if (fc >= 0.0 && fc < (double)w && fr >= 0.0 && fr < (double)h) {
                        int64_t idx = (int64_t)fr * w + (int64_t)fc;
                        a = wall_edt[idx];
                        b = blk_edt[idx];
                    }
                    if (j == 0 || a < vw) vw = a;
                    if (j == 0 || b < vu) vu = b;
                }
                float cw = vw - r32, cu = vu - r32;
                if (i == 0 || cw < wmin) wmin = cw;
                if (i == 0 || cu < umin) umin = cu;
                wlast = cw;
            }
            pose[3 * child] = x;
            pose[3 * child + 1] = y;
            pose[3 * child + 2] = yaw;
            wall_min[child] = wmin;
            unk_min[child] = umin;
            wall_end[child] = wlast;
        }
    }
}

/* 経路（区間 `nseg` 個）を `step` ごとに標本化し（`reeds_shepp.sample_path_array`、始点を含む）、
 * 各姿勢に車体を置いた余裕の最小（`LocalMap.path_clearance`、float32）を返す。Python 版と同じ式・同じ型 */
float hy_path_clearance(double x, double y, double yaw, int64_t nseg, const int64_t *gear,
                        const double *curv, const double *len, double step, const float *edt,
                        int64_t h, int64_t w, double o0, double o1, double res, int64_t k,
                        const double *cx, const double *cy, float r32)
{
    float best = 0.0f;
    int first = 1;
    for (int64_t si = -1; si < nseg; si++) {
        int64_t n = 1;
        double ds = 0.0, c0 = 0.0, s0 = 0.0;
        double x0 = x, y0 = y, yaw0 = yaw, r = 0.0;
        int straight = 1;
        if (si >= 0) {
            double q = ceil(len[si] / step);
            n = q > 1.0 ? (int64_t)q : 1;
            ds = (len[si] / (double)n) * (double)gear[si];
            straight = fabs(curv[si]) < 1e-9;
            c0 = cos(yaw0);
            s0 = sin(yaw0);
            if (!straight) r = 1.0 / curv[si];
        }
        for (int64_t i = 1; i <= n; i++) {
            double px, py, pyaw;
            if (si < 0) {                       /* 始点そのもの */
                px = x0; py = y0; pyaw = yaw0;
            } else {
                double sv = ds * (double)i;
                if (straight) {
                    px = x0 + sv * c0;
                    py = y0 + sv * s0;
                    pyaw = yaw0;
                } else {
                    double dyaw = sv * curv[si];
                    px = x0 + r * (sin(yaw0 + dyaw) - s0);
                    py = y0 - r * (cos(yaw0 + dyaw) - c0);
                    pyaw = yaw0 + dyaw;
                }
                x = px; y = py; yaw = pyaw;     /* 次の区間はこの区間の最後の姿勢から */
            }
            double c = cos(pyaw), s = sin(pyaw);
            float v = 0.0f;
            for (int64_t j = 0; j < k; j++) {
                double qx = px + cx[j] * c - cy[j] * s;
                double qy = py + cx[j] * s + cy[j] * c;
                double fc = floor((qx - o0) / res), fr = floor((qy - o1) / res);
                float a = 0.0f;
                if (fc >= 0.0 && fc < (double)w && fr >= 0.0 && fr < (double)h)
                    a = edt[(int64_t)fr * w + (int64_t)fc];
                if (j == 0 || a < v) v = a;
            }
            float cl = v - r32;
            if (first || cl < best) { best = cl; first = 0; }
        }
    }
    return best;
}

/* ── Hybrid A* の格子のヒューリスティック（`_Heuristic2D._dijkstra`） ──
 * 8近傍の Dijkstra。Python 版の heapq は (費用, 行, 列) の辞書式で取り出すので、同じ順位の付け方の
 * 二分ヒープなら取り出しの順が同じになり、緩和の順（近傍の並び）も同じなので費用の配列も同じになる */
typedef struct { double c; int64_t r, k; } hy_item;

static inline int hy_less(const hy_item *a, const hy_item *b)
{
    if (a->c != b->c) return a->c < b->c;
    if (a->r != b->r) return a->r < b->r;
    return a->k < b->k;
}

void hy_dijkstra(int64_t n, const uint8_t *free_, double straight, double diag, int64_t gr,
                 int64_t gc, double *cost)
{
    int64_t cap = 1024, sz = 0;
    hy_item *hp = malloc(sizeof(hy_item) * (size_t)cap);
    const int64_t dr[8] = {-1, 1, 0, 0, -1, -1, 1, 1}, dc[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    const double wt[8] = {straight, straight, straight, straight, diag, diag, diag, diag};
    cost[gr * n + gc] = 0.0;
    hp[sz++] = (hy_item){0.0, gr, gc};
    while (sz) {
        hy_item top = hp[0];
        hy_item last = hp[--sz];
        int64_t i = 0;
        while (1) {                                   /* 根を抜いて最後の要素を沈める */
            int64_t a = 2 * i + 1, b = a + 1, m = i;
            const hy_item *cur = &last;
            if (a < sz && hy_less(&hp[a], cur)) { m = a; cur = &hp[a]; }
            if (b < sz && hy_less(&hp[b], cur)) { m = b; }
            if (m == i) break;
            hp[i] = hp[m];
            i = m;
        }
        if (sz) hp[i] = last;
        double c = top.c;
        int64_t r0 = top.r, c0 = top.k;
        if (c > cost[r0 * n + c0]) continue;
        for (int d = 0; d < 8; d++) {
            int64_t r1 = r0 + dr[d], c1 = c0 + dc[d];
            if (!(r1 >= 0 && r1 < n && c1 >= 0 && c1 < n) || !free_[r1 * n + c1]) continue;
            double nc = c + wt[d];
            if (nc < cost[r1 * n + c1]) {
                cost[r1 * n + c1] = nc;
                if (sz == cap) { cap *= 2; hp = realloc(hp, sizeof(hy_item) * (size_t)cap); }
                int64_t j = sz++;                     /* 浮かせる */
                hy_item it = {nc, r1, c1};
                while (j > 0) {
                    int64_t p = (j - 1) / 2;
                    if (!hy_less(&it, &hp[p])) break;
                    hp[j] = hp[p];
                    j = p;
                }
                hp[j] = it;
            }
        }
    }
    free(hp);
}

/* ── 駐車の局所地図（`raspi/nav/local_map.py`・`grid.py`）の毎周期の処理 ──
 * numpy 版は 600×600 の格子全体に十数回の演算をかけていて、Pi で1周期 約20ms かかっていた。同じ判定を
 * 1回の走査でする（結果は同じ） */

/* `OccGrid.wall_mask`・`known_free_mask` と、`LocalMap.refresh` の距離変換の入力
 * （`(~wall_only).astype(uint8)`・`(~blocked).astype(uint8)`。外周1セルは塞ぐ）。`fresh` は NULL か今周期の当たり */
void lm_masks(int64_t h, int64_t w, const uint16_t *hits, const uint16_t *misses, int64_t min_hits,
              int64_t min_seen, const uint8_t *fresh, uint8_t *wall, uint8_t *free_, uint8_t *src_wall,
              uint8_t *src_blk)
{
    for (int64_t r = 0; r < h; r++) {
        for (int64_t c = 0; c < w; c++) {
            int64_t i = r * w + c;
            int32_t hh = hits[i], seen = (int32_t)hits[i] + (int32_t)misses[i];
            uint8_t wl = hh >= min_hits && hh * 2 >= seen;
            uint8_t fr = seen >= min_seen && hh * 2 < seen;
            wall[i] = wl;
            free_[i] = fr;
            uint8_t wf = wl || (fresh && fresh[i]);
            int border = r == 0 || r == h - 1 || c == 0 || c == w - 1;
            src_wall[i] = border ? 0 : !wf;
            src_blk[i] = border ? 0 : (!wf && fr);
        }
    }
}

/* `LocalMap._forget_vanished`（`n_hit` の判定の後）: 今周期に素通りされて当たりの無かった壁セル
 * （`hits > vanish`）を、今周期に当たったセルの 2 セル以内（8 近傍の膨張 2 回 = 5×5）を除いて `vanish` にする。
 * 戻り値: 素通りされたセルがあったか（numpy 版はそのときだけ読み出しのキャッシュを捨てる） */
int lm_forget(int64_t h, int64_t w, uint16_t *hits, const uint16_t *hits0, const uint16_t *misses,
              const uint16_t *miss0, uint16_t vanish)
{
    int64_t cap = 256, n = 0;
    int64_t *list = malloc(sizeof(int64_t) * (size_t)cap);
    for (int64_t i = 0; i < h * w; i++) {
        if (misses[i] > miss0[i] && !(hits[i] > hits0[i]) && hits[i] > vanish) {
            if (n == cap) { cap *= 2; list = realloc(list, sizeof(int64_t) * (size_t)cap); }
            list[n++] = i;
        }
    }
    /* 先に全部の判定を済ませてから書く（numpy 版も `near` は書き換える前の当たりで作る） */
    int64_t m = 0;
    for (int64_t q = 0; q < n; q++) {
        int64_t i = list[q], r = i / w, c = i % w, near = 0;
        for (int64_t rr = r - 2; rr <= r + 2 && !near; rr++) {
            if (rr < 0 || rr >= h) continue;
            for (int64_t cc = c - 2; cc <= c + 2; cc++) {
                if (cc < 0 || cc >= w) continue;
                int64_t j = rr * w + cc;
                if (hits[j] > hits0[j]) { near = 1; break; }
            }
        }
        if (!near) list[m++] = i;
    }
    for (int64_t q = 0; q < m; q++) hits[list[q]] = vanish;
    free(list);
    return n > 0;
}

/* `raspi/nav/grid.py` の `OccGrid._carve`（レイは距離の順に並べ替え済み）。`chunk` 本ずつの束ごとに、束の最遠の
 * レイの長さまで `step` 刻みの点（`t` は float32）を打ち、格子の中で `t < d − back` の点のセルを素通りとして
 * 1周に1回だけ数える（`_bump`、`count_max` で頭打ち）。束の全点が格子の外のレイの本数を返す（`out_of_bounds`）。
 * `touched` は h×w の 0 の作業領域（使った後に 0 へ戻す） */
int64_t rg_carve(int64_t n, double ox, double oy, const double *dx, const double *dy, const double *d,
                 int64_t chunk, double step, double back, double o0, double o1, double res, int64_t h,
                 int64_t w, uint16_t *misses, uint16_t count_max, uint8_t *touched)
{
    int64_t oob = 0, cap = 1024, nt = 0;
    int64_t *list = malloc(sizeof(int64_t) * (size_t)cap);
    float stepf = (float)step;
    for (int64_t s = 0; s < n; s += chunk) {
        int64_t e = s + chunk < n ? s + chunk : n;
        double q = ceil(d[e - 1] / step);
        int64_t ns = q > 1.0 ? (int64_t)q : 1;
        for (int64_t i = s; i < e; i++) {
            double lim = d[i] - back;
            int any_in = 0;
            for (int64_t k = 1; k <= ns; k++) {
                double tk = (double)((float)k * stepf);
                double px = ox + dx[i] * tk, py = oy + dy[i] * tk;
                double fc = floor((px - o0) / res), fr = floor((py - o1) / res);
                int in = fc >= 0.0 && fc < (double)w && fr >= 0.0 && fr < (double)h;
                if (in) any_in = 1;
                if (!(tk < lim)) {
                    if (any_in) break;          /* ここから先は数えない。外れの判定ももう要らない */
                    continue;
                }
                if (!in) continue;
                int64_t idx = (int64_t)fr * w + (int64_t)fc;
                if (!touched[idx]) {
                    touched[idx] = 1;
                    if (nt == cap) { cap *= 2; list = realloc(list, sizeof(int64_t) * (size_t)cap); }
                    list[nt++] = idx;
                }
            }
            if (!any_in) oob++;
        }
    }
    for (int64_t j = 0; j < nt; j++) {
        int64_t idx = list[j];
        if (misses[idx] < count_max) misses[idx] += 1;
        touched[idx] = 0;
    }
    free(list);
    return oob;
}

/* ── 点群どうしの照合（`raspi/nav/scan2scan.py`）の方位の表と対応探し ──
 * 方位のビン: ((atan2(y, x) / 2π + 1) · bins) を整数にして bins で割った余り（numpy 版と同じ式） */
static inline int64_t s2s_bin(double x, double y, int64_t bins)
{
    int64_t b = (int64_t)((atan2(y, x) / (2.0 * RS_PI) + 1.0) * (double)bins);
    return ((b % bins) + bins) % bins;
}

/* `_bearing_index`: 方位ごとにいちばん近い点の添字（無ければ −1）。`order` は numpy の `argsort(r)`（同じ距離の順を揃える） */
void s2s_bearing(int64_t n, const double *x, const double *y, const int64_t *order, int64_t bins,
                 int64_t *idx)
{
    double *best = malloc(sizeof(double) * (size_t)bins);
    for (int64_t b = 0; b < bins; b++) { idx[b] = -1; best[b] = INFINITY; }
    for (int64_t q = 0; q < n; q++) {
        int64_t i = order[q], b = s2s_bin(x[i], y[i], bins);
        double r = hypot(x[i], y[i]);
        if (r < best[b]) { best[b] = r; idx[b] = i; }
    }
    free(best);
}

/* `match_scans` の対応探し: 点 (qx, qy) の方位の前後 2 ビンの相手のうち、いちばん近いもの（添字と距離）。無ければ −1・inf */
void s2s_assoc(int64_t nq, const double *qx, const double *qy, const double *px, const double *py,
               const int64_t *lut, int64_t bins, int64_t *best_j, double *best_d)
{
    for (int64_t i = 0; i < nq; i++) {
        int64_t b = s2s_bin(qx[i], qy[i], bins), bj = -1;
        double bd = INFINITY;
        for (int64_t off = -2; off <= 2; off++) {
            int64_t j = lut[(((b + off) % bins) + bins) % bins];
            if (j < 0) continue;
            double d = hypot(qx[i] - px[j], qy[i] - py[j]);
            if (d < bd) { bd = d; bj = j; }
        }
        best_j[i] = bj;
        best_d[i] = bd;
    }
}
