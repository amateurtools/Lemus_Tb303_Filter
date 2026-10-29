#pragma once

#include <juce_audio_processors/juce_audio_processors.h>

#include <array>

#include "tb303_vcf.h"

// GUI-less wrapper around the standalone TB-303 VCF. The host draws its own parameter UI.
class TB303VCFAudioProcessor : public juce::AudioProcessor
{
public:
    TB303VCFAudioProcessor();
    ~TB303VCFAudioProcessor() override = default;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
    using AudioProcessor::processBlock;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override { return nullptr; }
    bool hasEditor() const override { return false; }

    const juce::String getName() const override { return JucePlugin_Name; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.5; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return "Init"; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();
    void updateLatency (int oversampleIndex);

    juce::AudioProcessorValueTreeState apvts;

    std::atomic<float>* cutoffParam = nullptr;
    std::atomic<float>* resonanceParam = nullptr;
    std::atomic<float>* inputDriveParam = nullptr;
    std::atomic<float>* outputGainParam = nullptr;
    std::atomic<float>* oversampleParam = nullptr;

    // One filter per oversampling setting (0 = off, 1 = 2x, 2 = 4x, 3 = 8x) and channel, all
    // prepared up front so switching never allocates on the audio thread. Only the active set runs.
    static constexpr int kNumOversampleModes = 4;
    static constexpr int kMaxChannels = 2;
    std::array<std::array<tb303vcf::Filter, kMaxChannels>, kNumOversampleModes> filters;
    int activeMode = 2;

    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Multiplicative> cutoffSmoothed;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> resonanceSmoothed;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> inputGainSmoothed;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> outputGainSmoothed;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TB303VCFAudioProcessor)
};