#include <AnalyzerChannelData.h>

#include <vector>
#include <algorithm>

#include "USBAnalyzer.h"
#include "USBAnalyzerSettings.h"

USBAnalyzer::USBAnalyzer() : Analyzer2(), mSimulationInitilized( false )
{
    SetAnalyzerSettings( &mSettings );
     UseFrameV2();
}

USBAnalyzer::~USBAnalyzer()
{
    KillThread();
}

void USBAnalyzer::SetupResults()
{
    // reset the results
    mResults.reset( new USBAnalyzerResults( this, &mSettings ) );
    SetAnalyzerResults( mResults.get() );

    // set which channels will carry bubbles
    mResults->AddChannelBubblesWillAppearOn( mSettings.mDPChannel );
    mResults->AddChannelBubblesWillAppearOn( mSettings.mDMChannel );
}

bool FrameLessThan( const Frame& lhs, const Frame& rhs )
{
    return lhs.mStartingSampleInclusive < rhs.mStartingSampleInclusive;
}

void USBAnalyzer::WorkerThread()
{
    // get the channel pointers
    mDP = GetAnalyzerChannelData( mSettings.mDPChannel );
    mDM = GetAnalyzerChannelData( mSettings.mDMChannel );

    USBSignalFilter sf( this, mResults.get(), &mSettings, mDP, mDM, mSettings.mSpeed );

    const double BIT_DUR = mSettings.mSpeed == FULL_SPEED ? FS_BIT_DUR : LS_BIT_DUR;
    const double SAMPLE_DUR = 1e9 / GetSampleRate(); // 1 sample duration in ns
    const double BIT_SAMPLES = BIT_DUR / SAMPLE_DUR;

    ResetUSB();

    USBSignalState s;
    USBPacket pckt;
    U64 lastFrameEnd = 0;
    while( sf.HasMoreData() )
    {
        s = sf.GetState();

        if( mSettings.mDecodeLevel == OUT_SIGNALS )
        {
            s.AddFrame( mResults.get() );
        }
        else
        {
            if( lastFrameEnd == 0 )
                lastFrameEnd = s.mSampleBegin;

            // if this is a data signal
            if( sf.IsDataSignal( s ) )
            {
                // try reading an entire USB packet by parsing subsequent data signals
                if( sf.GetPacket( pckt, s ) )
                {
                    if( mSettings.mDecodeLevel == OUT_PACKETS )
                    {
                        if( !mSettings.mExcludeSOF || (pckt.mPID != PID_SOF) )
                        {
                            lastFrameEnd = pckt.AddPacketFrames( mResults.get() );
                        }
                    }
                    else if( mSettings.mDecodeLevel == OUT_BYTES )
                    {
                        lastFrameEnd = pckt.AddRawByteFrames( mResults.get() );
                    }
                }
                else
                {
                    lastFrameEnd = pckt.AddErrorFrame( mResults.get() );
                }
            }
            else if( mSettings.mSpeed == LOW_SPEED // is this a LS Keep-alive?
                     && s.mState == S_SE0 && s.GetNumBits( LOW_SPEED ) == 2 )
            {
                Frame f;
                f.mStartingSampleInclusive = lastFrameEnd;
                f.mEndingSampleInclusive = s.mSampleEnd;
                f.mType = FT_KeepAlive;
                f.mFlags = FF_None;
                f.mData1 = f.mData2 = 0;

                mResults->AddFrame( f );
                mResults->CommitResults();

                lastFrameEnd = s.mSampleEnd;
            }
            else if( s.mState == S_SE0 && s.mDur > 1e7 )
            { // Reset?   dur > 10 ms

                Frame f;
                f.mStartingSampleInclusive = lastFrameEnd;
                f.mEndingSampleInclusive = s.mSampleEnd;
                f.mType = FT_Reset;
                f.mFlags = FF_None;
                f.mData1 = f.mData2 = 0;

                mResults->AddFrame( f );
                mResults->CommitResults();

                lastFrameEnd = s.mSampleEnd;

                ResetUSB();
            }
            else if( s.mState == S_J )
            { // Idle

                lastFrameEnd = s.mSampleEnd;
            }
        }

        ReportProgress( s.mSampleEnd );
        CheckIfThreadShouldExit();
    }
}

bool USBAnalyzer::NeedsRerun()
{
    return false;
}

U32 USBAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate,
                                         SimulationChannelDescriptor** simulation_channels )
{
    if( !mSimulationInitilized )
    {
        mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), &mSettings );
        mSimulationInitilized = true;
    }

    return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

U32 USBAnalyzer::GetMinimumSampleRateHz()
{
    return 24000000; // full 24MHz
}

const char* USBAnalyzer::GetAnalyzerName() const
{
    return ::GetAnalyzerName();
}

const char* GetAnalyzerName()
{
    return "USB LS and FS";
}

Analyzer* CreateAnalyzer()
{
    return new USBAnalyzer();
}

void DestroyAnalyzer( Analyzer* analyzer )
{
    delete analyzer;
}
