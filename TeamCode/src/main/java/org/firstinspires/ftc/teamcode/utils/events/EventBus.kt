package org.firstinspires.ftc.teamcode.utils.events

import java.util.LinkedList
import kotlin.reflect.KClass

class EventBus {
    private val _events = hashMapOf<KClass<*>, LinkedList<(Any) -> Unit>>()

    @Suppress("UNCHECKED_CAST")
    fun <T : Any> sub(event: KClass<T>, callback: (T) -> Unit) {
        var list = _events[event]

        if (list == null) {
            list = LinkedList()
            _events[event] = list
        }

        list.add(callback as (Any) -> Unit)
    }

    operator fun <T : Any> invoke(event: T): T {
        val list = _events[event::class] ?: return event

        for (i in list)
            i.invoke(event)

        return event
    }
}