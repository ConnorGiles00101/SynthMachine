/** Example of setting reading MIDI Input via USB Host
 *  
 * 
 *  This requires a USB-A connector
 * 
 *  This example will also log incoming messages to the serial port for general MIDI troubleshooting
 */
#include "daisy_seed.h"
#include "usbh_midi.h"
#include "daisysp.h"

/** This prevents us from having to type "daisy::" in front of a lot of things. */
using namespace daisy;
using namespace daisysp;

/** Global Hardware access */
DaisySeed      hw;
MidiUsbHandler midi;
USBHostHandle  usbHost;

// uint8_t output_buffer[8];
#define BUFF_SIZE 12
static uint8_t DMA_BUFFER_MEM_SECTION DAC_output_buffer[BUFF_SIZE];

#define MIDI_MAX 75
#define MIDI_MIN 28
#define I2C_SAMPLE_RATE 3000

class CustomAdsr {
    public:
        enum EnvState {
            Attack,
            Decay,
            Sustain,
            Release
        };

        enum NoteState {
            OnNote,
            OffNote
        };

        EnvState Env_State = Release;
        NoteState Note_Stat = OffNote;

        float sample_rate;

        float AttackTime;
        float DecayTime;
        float ReleaseTime;
        float SustainPoint;

        float ADSR_Sample_Val = 0;
        float ADSR_Sample_Val_old = 0;

        void ProcessEnvNoteOn (){
            
            switch(Env_State) {
                case Attack:
                    if (ADSR_Sample_Val >= 1){
                        ADSR_Sample_Val = 1;
                        Env_State = Decay;
                    } else {
                        ADSR_Sample_Val += 1 / (sample_rate*AttackTime);
                    }
                break;
                
                case Decay:
                    if (ADSR_Sample_Val <= SustainPoint){
                        ADSR_Sample_Val = SustainPoint;
                        Env_State = Sustain;
                    } else {
                        ADSR_Sample_Val -= (1-SustainPoint) / (sample_rate*DecayTime);
                    }
                break;

                case Sustain:
                    ADSR_Sample_Val = SustainPoint;
                break;

                case Release:
                    Env_State = Attack;
                    ADSR_Sample_Val = 0;
                break;
            }
            if (ADSR_Sample_Val >= 1){
                ADSR_Sample_Val = 1;
            }
            if (ADSR_Sample_Val <= 0){
                ADSR_Sample_Val = 0;
            }

        }

        void ProcessEnvNoteOff (){
                
                switch(Env_State) {
                    case Attack:
                        if (ADSR_Sample_Val >= 1){
                            ADSR_Sample_Val = 1;
                        }
                        ADSR_Sample_Val_old = ADSR_Sample_Val;
                        Env_State = Release;
                    break;
                    
                    case Decay:
                        ADSR_Sample_Val_old = ADSR_Sample_Val;
                        Env_State = Release;
                    break;

                    case Sustain:
                        ADSR_Sample_Val_old = ADSR_Sample_Val;
                        Env_State = Release;
                    break;

                    case Release:
                        ADSR_Sample_Val -= ADSR_Sample_Val_old / (sample_rate * ReleaseTime);
                    break;
                }

                if (ADSR_Sample_Val <= 0){
                    ADSR_Sample_Val = 0;
                }

            }

        void ProcessENV(){
            switch(Note_Stat){
                case OnNote:
                    ProcessEnvNoteOn();
                    break;
                case OffNote:
                    ProcessEnvNoteOff();
                    break; 
            }
        }

        void SetState(NoteState Note_State_Ext){
            Note_Stat = Note_State_Ext;
        }

        CustomAdsr(float Sample_Rate_Ext, float AttackTime_Ext, float DecayTime_Ext, float SustainPoint_Ext, float ReleaseTime_Ext){
            AttackTime = AttackTime_Ext;
            DecayTime = DecayTime_Ext;
            SustainPoint = SustainPoint_Ext;
            ReleaseTime = ReleaseTime_Ext;
            sample_rate = Sample_Rate_Ext;
        }
};

/** FIFO to hold messages as we're ready to print them */
FIFO<MidiEvent, 128> event_log;

void USBH_Connect(void* data)
{
    //hw.PrintLine("device connected");
}

void USBH_Disconnect(void* data)
{
    //hw.PrintLine("device disconnected");
}

void USBH_ClassActive(void* data)
{
    if(usbHost.IsActiveClass(USBH_MIDI_CLASS))
    {
        //hw.PrintLine("MIDI device class active");
        MidiUsbHandler::Config midi_config;
        midi_config.transport_config.periph = MidiUsbTransport::Config::Periph::HOST;
        midi.Init(midi_config);
        midi.StartReceive();
    }
}

void USBH_Error(void* data)
{
    //hw.PrintLine("USB device error");
}

uint8_t DAC_I2C_GENERATE_MULTI_CHANNEL_PARAMS(uint8_t VREF, uint8_t PD1, uint8_t PD0, uint8_t Gx){
    //generates channel top bits for multi-write, see SET VREF and SET GAIN and MULTI_WRITE, PD1 and PD0 should be set 0 with current hardware configuration
    uint8_t top_bits = ((VREF << 7) | (PD1 << 6) | (PD0 << 5) | (Gx << 4));
    return top_bits;
}

void DAC_I2C_MULTI_WRITE(uint8_t DAC_Address, uint8_t* DAC_top_bits, uint16_t* channels, uint8_t* output_buffer, uint8_t UDAC, I2CHandle I2C) { 
    //writes all DACS at once, channels stores output values, DAC_top_bits stores per-channel vref, gain, and power down
    for (uint8_t i = 0; i < 4; i++){
        if (channels[i] > 0xFFF){
            channels[i] = 0xFFF;
        }
        output_buffer[3*i] = static_cast<uint8_t>(0x40 | (i << 1) | UDAC);
        output_buffer[3*i+1] = static_cast<uint8_t>(DAC_top_bits[i] | channels[i] >> 8);
        output_buffer[3*i + 2] = static_cast<uint8_t>(channels[i] & 0xFF);
    }

    I2CHandle::Result Multi_Write_Result = I2C.TransmitDma(DAC_Address, &output_buffer[0], 12, NULL, NULL);
    if(Multi_Write_Result == I2CHandle::Result::OK) {
        //hw.PrintLine("OK TRANSMISSION Multi Write");
    }
}

void DAC_I2C_SET_VREF(uint8_t DAC_Address, uint8_t V_A, uint8_t V_B, uint8_t V_C, uint8_t V_D, uint8_t* output_buffer, I2CHandle I2C){
    //sets reference voltage for each channel, use at startup only, 0 = VDD, 1 = Internal Reference (2.048V)
    output_buffer[0] = (0x80 | (V_A << 3) | (V_B << 2) | (V_C << 1) | V_D); // command (4bits) Vref A Vref B Vref C Vref D
    I2CHandle::Result Vref_Write_Result = I2C.TransmitDma(DAC_Address, &output_buffer[0], 1, NULL, NULL);
    if(Vref_Write_Result == I2CHandle::Result::OK) {
        //hw.PrintLine("OK TRANSMISSION VREF Write");
    }
}

void DAC_I2C_SET_GAIN(uint8_t DAC_Address, uint8_t G_A, uint8_t G_B, uint8_t G_C, uint8_t G_D, uint8_t* output_buffer, I2CHandle I2C){
    //sets gain for each channel, use at startup only, 0 = gain of 1 (0-2.048 V if Vref internal), 1 = gain of 2 (0 - 4.096 V if Vref internal), if Vref = VDD set to 0
    output_buffer[0] = (0xC0 | (G_A << 3) | (G_B << 2) | (G_C << 1) | G_D); // command (4bits) Gx A Gx B Gx C Gx D
    I2CHandle::Result Gain_Write_Result = I2C.TransmitDma(DAC_Address, &output_buffer[0], 1, NULL, NULL);
    if(Gain_Write_Result == I2CHandle::Result::OK) {
        //hw.PrintLine("OK TRANSMISSION GAIN Write");
    }
}

uint16_t MIDI_NOTE_TO_DAC_VOLTAGE(uint8_t note){
    uint16_t MIDI_Range = MIDI_MAX - MIDI_MIN; 
    uint16_t DAC_Range = 4095;
    uint16_t DAC_Channel_Val = ((static_cast<uint16_t>(note) - MIDI_MIN) * DAC_Range) / MIDI_Range;

    return DAC_Channel_Val;
}

uint16_t OSC_AMP_TO_DAC_VOLTAGE(float value){
    float OSC_RANGE = 1.0f - (-1.0f); 
    float DAC_Range = 4095;
    float DAC_Channel_Val = ((value - (-1.0f)) * DAC_Range) / OSC_RANGE;

    return static_cast<uint16_t>(DAC_Channel_Val);
}

uint16_t Env_AMP_TO_DAC_VOLTAGE(float value){
    float Env_RANGE = 1.0f; 
    float DAC_Range = 4095;
    float DAC_Channel_Val = ((value) * DAC_Range) / Env_RANGE;

    return static_cast<uint16_t>(DAC_Channel_Val);
}

int main(void)
{
    /** Initialize our hardware */
    hw.Init();
    //hw.StartLog(true);

    //initialize Timer stuff
    TimerHandle timer;
    TimerHandle::Config T_CONFIG;
    T_CONFIG.dir = TimerHandle::Config::CounterDir::UP;
    T_CONFIG.periph = TimerHandle::Config::Peripheral::TIM_2;
    timer.Init(T_CONFIG);
    timer.Start();

    uint32_t freq = timer.GetFreq();
    uint32_t lastTime = timer.GetTick();
    float interval = 0;

    //hw.StartLog(true);

    //hw.PrintLine("MIDI USB Host start");

    /** Configure USB host */
    USBHostHandle::Config usbhConfig;
    usbhConfig.connect_callback = USBH_Connect;
    usbhConfig.disconnect_callback = USBH_Disconnect;
    usbhConfig.class_active_callback = USBH_ClassActive;
    usbhConfig.error_callback = USBH_Error;
    usbHost.Init(usbhConfig);

    usbHost.RegisterClass(USBH_MIDI_CLASS);

    float now      = 0;
    float log_time = 0;
    float blink_time = 0;
    bool ledState = false;
    float I2C_Signal_Clock = 0;
    uint8_t LED_INTER = 50;

    //hw.PrintLine("MIDI USB Host initialized");

    //configure the peripheral
    I2CHandle::Config _i2c_config;
    _i2c_config.periph = I2CHandle::Config::Peripheral::I2C_1;
    _i2c_config.speed  = I2CHandle::Config::Speed::I2C_400KHZ;
    _i2c_config.mode   = I2CHandle::Config::Mode::I2C_MASTER;
    _i2c_config.pin_config.scl  = seed::D11;
    _i2c_config.pin_config.sda  = seed::D12;
    // Address not needed for controller!

    // initialise the peripheral
    I2CHandle _i2c;
    _i2c.Init(_i2c_config);

    DAC_I2C_SET_VREF(0x60, 1, 1, 1, 1, &DAC_output_buffer[0], _i2c);
    DAC_I2C_SET_GAIN(0x60, 0, 0, 0, 0, &DAC_output_buffer[0], _i2c);

    uint16_t DAC_Channels[4] = {4095,3071,2047,1023};
        uint8_t DAC_Multi_Channel_Params[4] = {0,0,0,0};

        for (uint8_t i = 0; i < 4; i++){
            DAC_Multi_Channel_Params[i] = DAC_I2C_GENERATE_MULTI_CHANNEL_PARAMS(1,0,0,0);
        }

    /** initialize sine wave osc
    Oscillator sine_osc;
    sine_osc.Init(I2C_SAMPLE_RATE);
    sine_osc.SetFreq(100);
    sine_osc.SetWaveform(sine_osc.WAVE_SIN);
    sine_osc.SetAmp(1.0f);
    
    float sample = 0;
    */
    
    //initialize envelope
    CustomAdsr VolumeEnv(I2C_SAMPLE_RATE,0.1,0.1,0.5,0.1);

    /** Infinite Loop */
    while(1)
    {
        uint32_t newTick = timer.GetTick();
        interval = ((float)(newTick - lastTime) / (float)freq);
        lastTime = newTick;

        now += interval;
        blink_time += 1000.0 * interval;
        I2C_Signal_Clock += interval;

//Run USB host process
        usbHost.Process();

        if(usbHost.IsActiveClass(USBH_MIDI_CLASS) && midi.RxActive())
        {
            //Process MIDI in the background
            midi.Listen();

            //Loop through any MIDI Events
            while(midi.HasEvents())
            {
                MidiEvent msg = midi.PopEvent();

                //Handle messages as they come in 
                //See DaisyExamples for some examples of this
                
                switch(msg.type)
                {
                    case NoteOn:
                        // Do something on Note On events
                        {
                            //uint8_t VelID = msg.AsNoteOn().velocity;
                            DAC_Channels[0] = MIDI_NOTE_TO_DAC_VOLTAGE(msg.AsNoteOn().note);
                            VolumeEnv.SetState(CustomAdsr::OnNote);
                        }
                        break;

                    case NoteOff:
                    {
                        VolumeEnv.SetState(CustomAdsr::OffNote);
                    }
                    default: break;
                }

                //Regardless of message, let's add the message data to our queue to output
                event_log.PushBack(msg);
            }
/**
            //Now separately, every 5ms we'll print the top message in our queue if there is one
            if(now - log_time > 5)
            {
                log_time = now;
                if(!event_log.IsEmpty())
                {
                    auto msg = event_log.PopFront();
                    char outstr[128];
                    const char* type_str = MidiEvent::GetTypeAsString(msg);
                    sprintf(outstr,
                            "time:\t%f\ttype: %s\tChannel:  %d\tData MSB: "
                            "%d\tData LSB: %d\n",
                            now,
                            type_str,
                            msg.channel,
                            msg.data[0],
                            msg.data[1]);
                    //hw.PrintLine(outstr);
                }
            }*/
        }
        
        if(I2C_Signal_Clock >= (1.0f / I2C_SAMPLE_RATE)){
            I2C_Signal_Clock = 0;
            VolumeEnv.ProcessENV();

            DAC_Channels[2] = Env_AMP_TO_DAC_VOLTAGE(VolumeEnv.ADSR_Sample_Val);
            //hw.PrintLine("%d %d", OSC_AMP_TO_DAC_VOLTAGE(sample),static_cast<uint16_t>((sample+1)*10));
            DAC_I2C_MULTI_WRITE(0x60,DAC_Multi_Channel_Params,DAC_Channels,DAC_output_buffer,1,_i2c);
        }

        if (blink_time > LED_INTER)
        {
            blink_time = 0;
            hw.SetLed(ledState);
            ledState = !ledState;
            if (usbHost.GetPresent())
                LED_INTER = 50;
            else
                LED_INTER = 25;
        }

    }
}
