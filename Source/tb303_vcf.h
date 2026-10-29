// tb303_vcf.h -- the TB-303 VCF on its own. Header-only, no dependencies beyond the C++ standard
// library. Extracted from the voice model: the four-capacitor transistor ladder (18/33/33/33 nF),
// the Q21 output amp, the resonance feedback chain with its in-loop coupling high-passes, solved
// implicitly (trapezoidal + Newton on all four cap states).
//
// Usage:
//   tb303vcf::Filter f;
//   f.prepare(sampleRate, 2);          // 2 = 4x oversampling (0 = none, 1 = 2x, 3 = 8x)
//   f.setCutoffHz(800);                // ring (self-oscillation) frequency, see note below
//   f.setResonance(0.7);               // pot rotation 0..1
//   float y = f.process(x);            // per sample, or f.processBlock(in, out, n)
//
// Notes
//  * "Cutoff" here is the frequency the ladder rings at with resonance at max. It is set by the
//    ladder bias current: I = 2*pi*f * 2*Vt*33nF / 0.692, soft-limited at 0.65 mA (~42 kHz).
//  * The ladder is a nonlinear (tanh) circuit, so the input LEVEL matters. The original voice
//    feeds it ~+/-26 mV (a saw through R62 220k into R70 2.2k). setInputPeakVolts() maps a +/-1
//    input to that many volts at the Q12 base; raise it to drive the ladder harder.
//  * Output is the VCF OUT node in volts times outputGain (default 0.2, about +/-0.5 for a
//    full-scale saw at moderate settings).
//  * Component values use the "fitted unit" calibration (q21Beta 385, tail 0.52 mA).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace tb303vcf {

namespace detail {
constexpr double kVt = 0.025693; // thermal voltage at 25 C
constexpr double kPi = 3.14159265358979323846;

// One-pole high-pass (trapezoidal), split into y = a*x + b form for use inside implicit loops.
struct HighPass {
    double g = 0, s = 0, ga = 1;
    void set(double fc, double fs)
    {
        g = std::tan(kPi * std::min(fc, 0.45 * fs) / fs);
        ga = 1 / (1 + g);
    }
    double a() const { return ga; }
    double b() const { return -s * ga; }
    double process(double x)
    {
        const double v = (x - s) * g / (1 + g);
        const double lp = v + s;
        s = lp + v;
        return x - lp;
    }
    void reset() { s = 0; }
};

inline double besselI0(double x)
{
    double sum = 1, term = 1;
    for (int k = 1; k < 50; ++k) {
        term *= (x / (2 * k)) * (x / (2 * k));
        sum += term;
        if (term < 1e-12 * sum)
            break;
    }
    return sum;
}

// Kaiser-windowed half-band FIR, symmetric, stored as its odd taps. Length 4n-1.
struct HalfBandFir {
    int n = 0, len = 0, pos = 0;
    std::vector<double> taps, buf;

    void design(int halfLength, double attenuationDb)
    {
        n = halfLength;
        len = 4 * n - 1;
        const double beta = attenuationDb > 50 ? 0.1102 * (attenuationDb - 8.7)
                                               : 0.5842 * std::pow(attenuationDb - 21, 0.4) + 0.07886 * (attenuationDb - 21);
        const double span = 2.0 * n;
        taps.assign(size_t(n), 0.0);
        double sum = 0.5;
        for (int k = 0; k < n; ++k) {
            const double m = 2 * k + 1;
            const double sinc = std::sin(kPi * m / 2) / (kPi * m);
            const double w = besselI0(beta * std::sqrt(std::max(0.0, 1 - (m / span) * (m / span)))) / besselI0(beta);
            taps[size_t(k)] = sinc * w;
            sum += 2 * taps[size_t(k)];
        }
        for (double &t : taps)
            t /= sum; // unity DC gain
        buf.assign(size_t(2 * len), 0.0);
        pos = 0;
    }
    void reset()
    {
        std::fill(buf.begin(), buf.end(), 0.0);
        pos = 0;
    }
    void write(double x)
    {
        pos = pos == 0 ? len - 1 : pos - 1;
        buf[size_t(pos)] = buf[size_t(pos + len)] = x;
    }
    double compute() const
    {
        const double *h = &buf[size_t(pos)]; // h[0] newest
        const int c = 2 * n - 1;
        double y = 0.5 * h[c];
        for (int k = 0; k < n; ++k) {
            const int m = 2 * k + 1;
            y += taps[size_t(k)] * (h[c - m] + h[c + m]);
        }
        return y;
    }
};
} // namespace detail

// ------------------------------------------------------------------------------------------
// The ladder itself, running at whatever rate you give it (use it directly if you oversample
// yourself). Input and output are in volts.
struct CoreConfig {
    double q21Beta = 385;     // sets C25's in-loop high-pass corner and the load on C26
    double q21TailmA = 0.52;  // Q20 tail current: output amp gain and limiting
};

class Core {
public:
    using Config = CoreConfig;

    void prepare(double rate, const Config &c = Config())
    {
        fs = rate;
        h = 1 / fs;
        cfg = c;
        // C25/C27 0.1 uF into R113/R114 100k in parallel with Q21's r_pi.
        const double iSide = cfg.q21TailmA * 1e-3 / 2;
        const double rpi = cfg.q21Beta * detail::kVt / iSide;
        const double req = 1 / (1 / 100e3 + 1 / rpi);
        hpC25.set(1 / (2 * detail::kPi * 0.1e-6 * req), fs);
        rLoad = 2 * req;
        // Gain trims fitted against SPICE of the schematic (Q21 output resistance, Q19/Q18
        // followers, R67 into Q18).
        const double rCollector = 1 / (1 / 22e3 + iSide / 100.0 + 1 / 2.4e6);
        ampGain = rCollector * iSide * 0.994 * 0.975;
        loopFollower = 0.989 * 0.977;
        hpC14.set(1 / (2 * detail::kPi * 1e-6 * 50e3), fs);   // 3.2 Hz
        hpC15.set(1 / (2 * detail::kPi * 1e-6 * 98e3), fs);   // 1.6 Hz
        hpC23.set(1 / (2 * detail::kPi * 1e-6 * 12.2e3), fs); // 13 Hz
        setResonance(resonance);
        reset();
    }

    void reset()
    {
        for (int i = 0; i < 4; ++i)
            u[i] = fPrev[i] = fPrev2[i] = 0;
        hpC25.reset();
        hpC14.reset();
        hpC15.reset();
        hpC23.reset();
        vcfOut = vcfWiper = 0;
    }

    // Resonance pot rotation 0..1 (VR4-A, loaded by ~50k).
    void setResonance(double r)
    {
        resonance = std::clamp(r, 0.0, 1.0);
        const double kr = resonance, rl = 50e3;
        resWiper = kr * rl / (rl + kr * (1 - kr) * 50e3);
    }

    // Ladder tail current in amps (what the expo converter produces).
    static double currentForRingHz(double hz)
    {
        const double i = hz * 2 * detail::kPi * 2 * detail::kVt * 33e-9 / 0.692;
        constexpr double iMax = 0.65e-3; // Q11 saturation: soft limit
        return std::max(1e-9, i / std::sqrt(1 + (i / iMax) * (i / iMax)));
    }

    // One step. xIn: signal at the Q12 base (V). iF: ladder current (A). Returns VCF OUT (V, AC).
    double process(double xIn, double iF)
    {
        using detail::kVt;
        const double C[4] = {0.018e-6, 0.033e-6, 0.033e-6, 0.033e-6};
        constexpr double kInv2Vt = 1 / (2 * kVt);
        const double gTop = 1 / (rLoad * C[3]); // C26's discharge through Q21's input, per volt
        double kc[4];
        for (int i = 0; i < 4; ++i)
            kc[i] = iF * (0.5 / C[i]);

        // Linear parts of the feedback chain at this step:
        // w = aW*u4 + bW (C25/C27), amp = G tanh(w/2Vt), out = a14*amp + b14 (C14),
        // wiper = resWiper*out, q18 = a15*wiper + b15 (C15), fb = div*(a23*q18 + b23) (C23).
        const double aW = hpC25.a(), bW = hpC25.b();
        const double a14 = hpC14.a(), b14 = hpC14.b();
        const double a15 = hpC15.a(), b15 = hpC15.b();
        const double a23 = hpC23.a(), b23 = hpC23.b();
        constexpr double div = 2.2 / 12.2;
        const double A = div * a23 * loopFollower * a15 * resWiper * a14 * ampGain;
        const double B = div * (a23 * loopFollower * (a15 * resWiper * b14 + b15) + b23);
        auto feedback = [&](double u4, double &dfb) {
            const double w = aW * u4 + bW;
            const double t = std::tanh(w * kInv2Vt);
            dfb = A * (1 - t * t) * kInv2Vt * aW;
            return A * t + B;
        };

        // Trapezoidal step, Newton on u1..u4 from an Adams-Bashforth predictor.
        double un[4];
        for (int i = 0; i < 4; ++i)
            un[i] = u[i] + h * (1.5 * fPrev[i] - 0.5 * fPrev2[i]);
        const double hh = 0.5 * h;
        double x = 0, tc[5];
        bool converged = false;
        for (int it = 0; it < maxIterations; ++it) {
            double dfb;
            const double fb = feedback(un[3], dfb);
            x = xIn - fb;
            const double t0 = std::tanh(x * kInv2Vt);
            const double t1 = std::tanh((un[0] - un[1]) * kInv2Vt);
            const double t2 = std::tanh((un[1] - un[2]) * kInv2Vt);
            const double t3 = std::tanh((un[2] - un[3]) * kInv2Vt);
            const double t4 = std::tanh(un[3] * kInv2Vt);
            const double iLC = (aW * un[3] + bW) * gTop; // Q21's input also loads C26
            const double f[4] = {kc[0] * (t0 - t1), kc[1] * (t1 - t2), kc[2] * (t2 - t3), kc[3] * (t3 - t4) - iLC};
            double F[4];
            for (int i = 0; i < 4; ++i)
                F[i] = un[i] - u[i] - hh * (f[i] + fPrev[i]);
            const double d0 = (1 - t0 * t0) * kInv2Vt, d1 = (1 - t1 * t1) * kInv2Vt, d2 = (1 - t2 * t2) * kInv2Vt,
                         d3 = (1 - t3 * t3) * kInv2Vt, d4 = (1 - t4 * t4) * kInv2Vt;
            // df/du: tridiagonal, plus the feedback in row 0, column 3.
            const double j00 = -kc[0] * d1, j01 = kc[0] * d1, j03 = kc[0] * d0 * (-dfb);
            const double j10 = kc[1] * d1, j11 = -kc[1] * (d1 + d2), j12 = kc[1] * d2;
            const double j21 = kc[2] * d2, j22 = -kc[2] * (d2 + d3), j23 = kc[2] * d3;
            const double j32 = kc[3] * d3, j33 = -kc[3] * (d3 + d4) - aW * gTop;
            // Solve (I - hh J) d = -F by elimination on the nonzero entries (row 1 fills column 3).
            const double m00 = 1.0 - hh * j00, m01 = -hh * j01, m03 = -hh * j03;
            const double m10 = -hh * j10, m12 = -hh * j12;
            const double m21 = -hh * j21, m23 = -hh * j23;
            const double m32 = -hh * j32;
            double m11 = 1.0 - hh * j11, m22 = 1.0 - hh * j22, m33 = 1.0 - hh * j33;
            double d[4] = {-F[0], -F[1], -F[2], -F[3]};
            const double e1 = m10 / m00;
            m11 -= e1 * m01;
            const double m13 = -e1 * m03;
            d[1] -= e1 * d[0];
            const double e2 = m21 / m11;
            m22 -= e2 * m12;
            const double m23f = m23 - e2 * m13;
            d[2] -= e2 * d[1];
            const double e3 = m32 / m22;
            m33 -= e3 * m23f;
            d[3] -= e3 * d[2];
            d[3] = d[3] / m33;
            d[2] = (d[2] - m23f * d[3]) / m22;
            d[1] = (d[1] - m12 * d[2] - m13 * d[3]) / m11;
            d[0] = (d[0] - m01 * d[1] - m03 * d[3]) / m00;
            double mx = 0;
            for (int i = 0; i < 4; ++i) {
                un[i] += std::clamp(d[i], -0.1, 0.1);
                mx = std::max(mx, std::abs(d[i]));
            }
            if (mx < tolerance) {
                // First-order correction of the tanh terms to the updated state.
                tc[0] = t0;
                tc[1] = t1 + (1 - t1 * t1) * ((d[0] - d[1]) * kInv2Vt);
                tc[2] = t2 + (1 - t2 * t2) * ((d[1] - d[2]) * kInv2Vt);
                tc[3] = t3 + (1 - t3 * t3) * ((d[2] - d[3]) * kInv2Vt);
                tc[4] = t4 + (1 - t4 * t4) * (d[3] * kInv2Vt);
                converged = true;
                break;
            }
        }
        if (!converged) {
            tc[0] = std::tanh(x * kInv2Vt);
            tc[1] = std::tanh((un[0] - un[1]) * kInv2Vt);
            tc[2] = std::tanh((un[1] - un[2]) * kInv2Vt);
            tc[3] = std::tanh((un[2] - un[3]) * kInv2Vt);
            tc[4] = std::tanh(un[3] * kInv2Vt);
        }
        // Commit the step.
        const double t0 = tc[0], t1 = tc[1], t2 = tc[2], t3 = tc[3], t4 = tc[4];
        for (int i = 0; i < 4; ++i)
            fPrev2[i] = fPrev[i];
        fPrev[0] = kc[0] * (t0 - t1);
        fPrev[1] = kc[1] * (t1 - t2);
        fPrev[2] = kc[2] * (t2 - t3);
        fPrev[3] = kc[3] * (t3 - t4) - (hpC25.a() * un[3] + hpC25.b()) * gTop;
        for (int i = 0; i < 4; ++i)
            u[i] = un[i];
        const double w = hpC25.process(u[3]);
        const double amp = ampGain * std::tanh(w * kInv2Vt);
        vcfOut = hpC14.process(amp);
        vcfWiper = resWiper * vcfOut;
        const double q18 = hpC15.process(vcfWiper);
        hpC23.process(loopFollower * q18);
        return vcfOut;
    }

    double wiperVolts() const { return vcfWiper; } // resonance wiper (feeds the VCA's second branch)

    int maxIterations = 6;      // Newton iterations per sample
    double tolerance = 1e-10;   // convergence threshold on the state update (V)

private:
    double fs = 176400, h = 1 / 176400.0;
    Config cfg;
    double u[4] = {0, 0, 0, 0};
    double fPrev[4] = {0, 0, 0, 0}, fPrev2[4] = {0, 0, 0, 0};
    detail::HighPass hpC25, hpC14, hpC15, hpC23;
    double resonance = 0.5, resWiper = 0;
    double ampGain = 5.2, loopFollower = 0.966, rLoad = 55e3;
    double vcfOut = 0, vcfWiper = 0;
};

// ------------------------------------------------------------------------------------------
// Host-rate wrapper: oversamples (Kaiser half-band chain), runs the Core, decimates.
class Filter {
public:
    // oversampleStages: 0 = none, 1 = 2x, 2 = 4x (the original runs ~176 kHz internally), 3 = 8x.
    void prepare(double hostRate, int oversampleStages = 2, const Core::Config &cfg = Core::Config())
    {
        stages = std::clamp(oversampleStages, 0, 3);
        factor = 1 << stages;
        hostFs = hostRate;
        core.prepare(hostRate * factor, cfg);
        // Stage 0 sits next to the host rate and is the long, sharp one.
        up.assign(size_t(stages), {});
        down.assign(size_t(stages), {});
        for (int s = 0; s < stages; ++s) {
            const bool sharp = s == 0;
            up[size_t(s)].design(sharp ? 32 : 12, sharp ? 110.0 : 100.0);
            down[size_t(s)].design(sharp ? 32 : 12, sharp ? 110.0 : 100.0);
        }
        lnPrev = lnNow = std::log(Core::currentForRingHz(cutoffHz));
    }

    void reset()
    {
        core.reset();
        for (auto &b : up)
            b.reset();
        for (auto &b : down)
            b.reset();
        lnPrev = lnNow = std::log(Core::currentForRingHz(cutoffHz));
    }

    void setCutoffHz(double hz) { cutoffHz = std::clamp(hz, 5.0, 40000.0); }
    void setResonance(double r) { core.setResonance(r); }
    void setInputPeakVolts(double v) { inputPeakVolts = v; }
    void setOutputGain(double g) { outputGain = g; }
    Core &getCore() { return core; }

    // Round-trip latency of the oversampling filters, in host samples.
    double latencySamples() const
    {
        double lat = 0;
        for (int s = 0; s < stages; ++s)
            lat += 2.0 * (2.0 * up[size_t(s)].n - 1) / double(1 << (s + 1)); // up + down, centre tap at 2n-1
        return lat;
    }

    float process(float in)
    {
        lnPrev = lnNow;
        lnNow = std::log(Core::currentForRingHz(cutoffHz));

        // Upsample: sig[] holds `factor` samples after the last stage.
        double sig[8];
        sig[0] = in;
        int count = 1;
        for (int s = 0; s < stages; ++s) {
            double nxt[8];
            for (int i = 0; i < count; ++i) {
                auto &f = up[size_t(s)];
                f.write(sig[i]);
                nxt[2 * i] = 2.0 * f.compute();
                f.write(0.0);
                nxt[2 * i + 1] = 2.0 * f.compute();
            }
            count *= 2;
            for (int i = 0; i < count; ++i)
                sig[i] = nxt[i];
        }

        double out[8];
        for (int j = 0; j < factor; ++j) {
            const double frac = double(j + 1) / factor;
            const double iF = std::exp(lnPrev + (lnNow - lnPrev) * frac);
            double y = core.process(sig[j] * inputPeakVolts, iF) * outputGain;
            if (!std::isfinite(y)) {
                core.reset();
                y = 0;
            }
            out[j] = y;
        }

        // Decimate, highest rate first.
        count = factor;
        for (int s = stages - 1; s >= 0; --s) {
            auto &f = down[size_t(s)];
            for (int i = 0; i < count / 2; ++i) {
                f.write(out[2 * i]);
                f.write(out[2 * i + 1]);
                out[i] = f.compute();
            }
            count /= 2;
        }
        return float(out[0]);
    }

    void processBlock(const float *in, float *out, int n)
    {
        for (int i = 0; i < n; ++i)
            out[i] = process(in[i]);
    }

private:
    Core core;
    std::vector<detail::HalfBandFir> up, down;
    int stages = 2, factor = 4;
    double hostFs = 44100;
    double cutoffHz = 1000;
    double inputPeakVolts = 0.026;
    double outputGain = 0.2;
    double lnPrev = 0, lnNow = 0;
};

} // namespace tb303vcf