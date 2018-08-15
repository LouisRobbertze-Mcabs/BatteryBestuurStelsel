/*
 * User_Queue.h
 *
 *  Created on: 14 Mar 2017
 *      Author: Sonja
 */

#ifndef INCLUDE_USER_QUEUE_H_
#define INCLUDE_USER_QUEUE_H_

#include "User_Defines.h"
#include <string.h>

#define MAX_QUEUE_SIZE 80

// Queue functions

struct queue_obj
{
    Uint32 queue[MAX_QUEUE_SIZE][4];
    int front;
    int rear;
    int itemCount;
};

Uint32 queue_peek(struct queue_obj queue);
int is_queue_empty(struct queue_obj queue);
int is_queue_full(struct queue_obj queue);
int queue_size(struct queue_obj queue);
void queue_insert(Uint16 Dest, Uint32 data_H, Uint32 data_L, Uint16 Bytes, struct queue_obj* queue_obj);
void queue_remove_data(struct queue_obj* queue/*, Uint32* Value, Uint32* Data*/);

#endif /* INCLUDE_USER_QUEUE_H_ */
