/*
 * User_queue.c
 *
 *  Created on: 14 Mar 2017
 *      Author: Sonja
 */

#include "User_Queue.h"

Uint32 peek_queue(struct queue_obj queue){
    return queue.front;
}

int is_queue_empty(struct queue_obj queue){
    return queue.itemCount == 0;
}

int is_queue_full(struct queue_obj queue){
    return queue.itemCount == MAX_QUEUE_SIZE;
}

int queue_size(struct queue_obj queue){
    return queue.itemCount;
}

void queue_insert(Uint16 Dest, Uint32 data_H, Uint32 data_L, Uint16 Bytes, struct queue_obj* queue_obj){
    if(!is_queue_full(*queue_obj)) {
        if(queue_obj->rear == MAX_QUEUE_SIZE-1) {
            queue_obj->rear = -1;
        }
        queue_obj->queue[++queue_obj->rear][0] = Dest;
        queue_obj->queue[queue_obj->rear][1] = data_H;
        queue_obj->queue[queue_obj->rear][2] = data_L;
        queue_obj->queue[queue_obj->rear][3] = Bytes;
        queue_obj->itemCount++;
    }
}

void queue_remove_data(struct queue_obj* queue_obj/*, Uint32* Value, Uint32* Data*/){
  /*  *Value = queue_obj->queue[queue_obj->front][0];
    *Data = queue_obj->queue[queue_obj->front++][1];*/

    queue_obj->front++;

    if(queue_obj->front == MAX_QUEUE_SIZE) {
        queue_obj->front = 0;
    }
    queue_obj->itemCount--;
}
