#include <stdint.h>
#include "TeensyThreads.h"

struct SampleEntry {
    void (*func)();
    uint32_t delay;
    uint32_t last_sampled;
    SampleEntry* next;
};

class Sampler {
public:
    void insertFunction(void (*func)(), float frequency) {
        SampleEntry* new_entry = new SampleEntry(); // this is never freed :)
        new_entry->func = func;
        new_entry->delay = 1 / frequency * 1000000; //micros
        new_entry->last_sampled = 0;
        insertEntry(new_entry);
    }

    void insertEntry(SampleEntry* new_entry) {
        new_entry->next = nullptr;
        SampleEntry** entry = &first_entry;
        while (*entry != nullptr) {
            entry = &((*entry)->next);
        }
        *entry = new_entry;
    }

    void update(uint32_t dt) {
        /*
        clock_divider_counter += dt;
        uint32_t div = clock_divider_counter / clock_divider;
        clock += div;
        clock_divider_counter -= div * clock_divider_counter;
        */
       clock += dt;
        SampleEntry* entry = first_entry;
        while (entry != nullptr) {
            if (clock - entry->last_sampled >= entry->delay * clock_divider) {
                entry->last_sampled = clock;
                threads.addThread(entry->func);
            }
            entry = entry->next;
        }
    }

    void setClockDivider(uint32_t divider) {
        clock_divider = divider;
    }

private:
    SampleEntry* first_entry;
    uint32_t clock;
    uint32_t clock_divider = 1;
    uint32_t clock_divider_counter;
};