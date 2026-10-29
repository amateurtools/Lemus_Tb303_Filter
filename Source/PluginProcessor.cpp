#include "PluginProcessor.h"

#include <cmath>

namespace
{
// Level at the Q12 base that a full-scale (+/-1) input maps to, at 0 dB input drive. This is the
// signal the original voice feeds the ladder (a saw through R62/R70), see tb303_vcf.h.
constexpr double kBaseInputPeakVolts = 0.026;
// The filter's own output scale (VCF OUT volts -> sample value) at 0 dB output gain.
constexpr double kBaseOutputScale = 0.2;
constexpr float kSmoothSeconds = 0.02f;

inline float dbToLin (float db) { return std::pow (10.0f, db / 20.0f); }
} // namespace

TB303VCFAudioProcessor::TB303VCFAudioProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                          .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "Parameters", createParameterLayout())
{
    cutoffParam     = apvts.getRawParameterValue ("cutoff");
    resonanceParam  = apvts.getRawParameterValue ("resonance");
    inputDriveParam = apvts.getRawParameterValue ("inputDrive");
    outputGainParam = apvts.getRawParameterValue ("outputGain");
    oversampleParam = apvts.getRawParameterValue ("oversampling");
}

juce::AudioProcessorValueTreeState::ParameterLayout TB303VCFAudioProcessor::createParameterLayout()
{
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> params;

    // Frequency the ladder rings at with resonance at max (not a -3 dB point).
    {
        juce::NormalisableRange<float> range (20.0f, 20000.0f, 0.1f);
        range.setSkewForCentre (1000.0f);
        params.push_back (std::make_unique<juce::AudioParameterFloat> (
            juce::ParameterID { "cutoff", 1 }, "Cutoff", range, 1000.0f,
            juce::AudioParameterFloatAttributes().withLabel ("Hz")));
    }

    params.push_back (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "resonance", 1 }, "Resonance",
        juce::NormalisableRange<float> (0.0f, 1.0f, 0.001f), 0.5f));

    // Drives the ladder's tanh stages. 0 dB = the level the original voice's saw delivers.
    params.push_back (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "inputDrive", 1 }, "Input Drive",
        juce::NormalisableRange<float> (-24.0f, 24.0f, 0.1f), 0.0f,
        juce::AudioParameterFloatAttributes().withLabel ("dB")));

    params.push_back (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "outputGain", 1 }, "Output Gain",
        juce::NormalisableRange<float> (-24.0f, 24.0f, 0.1f), 0.0f,
        juce::AudioParameterFloatAttributes().withLabel ("dB")));

    // Internal oversampling of the nonlinear ladder (the original voice runs ~176 kHz internally).
    params.push_back (std::make_unique<juce::AudioParameterChoice> (
        juce::ParameterID { "oversampling", 1 }, "Oversampling",
        juce::StringArray { "Off", "2x", "4x", "8x" }, 2));

    return { params.begin(), params.end() };
}

void TB303VCFAudioProcessor::prepareToPlay (double sampleRate, int /*samplesPerBlock*/)
{
    for (int mode = 0; mode < kNumOversampleModes; ++mode)
        for (auto& f : filters[(size_t) mode])
            f.prepare (sampleRate, mode);

    activeMode = juce::jlimit (0, kNumOversampleModes - 1, (int) oversampleParam->load());
    updateLatency (activeMode);

    cutoffSmoothed.reset (sampleRate, kSmoothSeconds);
    resonanceSmoothed.reset (sampleRate, kSmoothSeconds);
    inputGainSmoothed.reset (sampleRate, kSmoothSeconds);
    outputGainSmoothed.reset (sampleRate, kSmoothSeconds);

    cutoffSmoothed.setCurrentAndTargetValue (cutoffParam->load());
    resonanceSmoothed.setCurrentAndTargetValue (resonanceParam->load());
    inputGainSmoothed.setCurrentAndTargetValue (dbToLin (inputDriveParam->load()));
    outputGainSmoothed.setCurrentAndTargetValue (dbToLin (outputGainParam->load()));
}

void TB303VCFAudioProcessor::updateLatency (int oversampleIndex)
{
    setLatencySamples ((int) std::lround (filters[(size_t) oversampleIndex][0].latencySamples()));
}

bool TB303VCFAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto out = layouts.getMainOutputChannelSet();
    if (out != juce::AudioChannelSet::mono() && out != juce::AudioChannelSet::stereo())
        return false;
    return layouts.getMainInputChannelSet() == out;
}

void TB303VCFAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;

    const int numChannels = juce::jmin (buffer.getNumChannels(), kMaxChannels);
    const int numSamples = buffer.getNumSamples();

    for (int ch = getTotalNumInputChannels(); ch < getTotalNumOutputChannels(); ++ch)
        buffer.clear (ch, 0, numSamples);

    // Oversampling change: switch to the (already prepared) filter set, flushed so it starts clean.
    const int newMode = juce::jlimit (0, kNumOversampleModes - 1, (int) oversampleParam->load());
    if (newMode != activeMode)
    {
        activeMode = newMode;
        for (auto& f : filters[(size_t) activeMode])
            f.reset();
        updateLatency (activeMode);
    }
    auto& set = filters[(size_t) activeMode];

    cutoffSmoothed.setTargetValue (juce::jlimit (20.0f, 20000.0f, cutoffParam->load()));
    resonanceSmoothed.setTargetValue (resonanceParam->load());
    inputGainSmoothed.setTargetValue (dbToLin (inputDriveParam->load()));
    outputGainSmoothed.setTargetValue (dbToLin (outputGainParam->load()));

    float* data[kMaxChannels] = { nullptr, nullptr };
    for (int ch = 0; ch < numChannels; ++ch)
        data[ch] = buffer.getWritePointer (ch);

    for (int i = 0; i < numSamples; ++i)
    {
        const double cutoff = cutoffSmoothed.getNextValue();
        const double res = resonanceSmoothed.getNextValue();
        const double inPeak = kBaseInputPeakVolts * inputGainSmoothed.getNextValue();
        const float outGain = (float) (kBaseOutputScale * outputGainSmoothed.getNextValue());

        for (int ch = 0; ch < numChannels; ++ch)
        {
            auto& f = set[(size_t) ch];
            f.setCutoffHz (cutoff);
            f.setResonance (res);
            f.setInputPeakVolts (inPeak);
            f.setOutputGain (1.0); // output scaling is applied below, together with the smoothed gain
            data[ch][i] = f.process (data[ch][i]) * outGain;
        }
    }
}

void TB303VCFAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    auto state = apvts.copyState();
    std::unique_ptr<juce::XmlElement> xml (state.createXml());
    copyXmlToBinary (*xml, destData);
}

void TB303VCFAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    std::unique_ptr<juce::XmlElement> xmlState (getXmlFromBinary (data, sizeInBytes));
    if (xmlState != nullptr && xmlState->hasTagName (apvts.state.getType()))
        apvts.replaceState (juce::ValueTree::fromXml (*xmlState));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new TB303VCFAudioProcessor();
}