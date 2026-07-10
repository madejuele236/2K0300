#ifndef PRIMER_CODE_PORT_LOW_PASS_FILTER_HPP_
#define PRIMER_CODE_PORT_LOW_PASS_FILTER_HPP_

namespace primer::port {

inline void ApplyLowPass(float hz, float time, float in, float *out)
{
    float alpha = 1 / (1 + 1 / (hz * 6.28f * time));

    *out += alpha * (in - *out);
}

}  // namespace primer::port

#endif
