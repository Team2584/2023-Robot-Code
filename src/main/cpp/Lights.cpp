class lightStrip
{
    lightStrip(int length, int port)
    {
        private:
            static constexpr int kLength = length;

            frc::AddressableLED m_led{port};
            std::array<frc::AddressableLED::LEDData, kLength>
                m_ledBuffer;  // Reuse the buffer
            // Store what the last hue of the first pixel is
            m_led.SetLength(length);
            m_led.SetData(m_ledBuffer);
            m_led.Start();
        fill(0, 255, 0);
    }

    void fill(int h, int s, int v){
        for (int i; i < kLength; i++){
            m_ledBuffer[i].SetHSV(h, s, v)
        }
        m_led.SetData(m_ledBuffer);
    }

    void effect(int minH, int maxH, int minS, int maxS, int minV, int maxV){

    }
}