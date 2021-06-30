//
// Created by Leonard Koll on 30.06.21.
//

#include "StateMachine.h"

template<typename T, int size>
StateMachine<T, size>::StateMachine(array<T*, size> states, array<int, size> default_transitions) :
    states{states}, default_transitions{default_transitions}
{}

template<typename T, int size>
void StateMachine<T, size>::next() {
    active_index = default_transitions[active_index];
}

template<typename T, int size>
T StateMachine<T, size>::get_active() {
    return *states[active_index];
}

template<typename T, int size>
void StateMachine<T, size>::set_active(int index) {
    if (index >= 0 && index < size) {
        active_index = index;
    }
}

