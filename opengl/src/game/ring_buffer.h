#pragma once

struct RingBuffer {
    static constexpr int SLOTS = 5;
    int buffer[SLOTS] = { 0 };
    int index = 0;
    int count = 0;

    // Ska köras varje frame med det nya värdet:
    void push(int newCount) {
        buffer[index] = newCount;
        index = (index + 1) % SLOTS;
        if (count < SLOTS) ++count;
    }

    // Ger medelvärdet över de senast count värdena (upp till 5)
    float average() const {
        if (count == 0) return 0.0f;

        int sum = 0;
        for (int i = 0; i < count; ++i) {
            sum += buffer[i];
        }
        return (float)sum / (float)count;
    }
};