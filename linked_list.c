
#include <stdio.h>
#include <stdlib.h>
#include "linked_list.h"

void Insert(LinkedList* list, int x, int y) {
    struct ListNode *temp = (struct ListNode*)malloc(sizeof(struct ListNode));
    temp->val = x;
    temp->angle = y;
    temp->next = NULL;
    if (list->head == NULL) list->head = temp;
    else {
        struct ListNode *temp2 = list->head;
        while (temp2->next != NULL) temp2 = temp2->next;
        temp2->next = temp;
    }
}

void Print(LinkedList* list) {
    struct ListNode *temp = list->head;
    while (temp != NULL) {
        printf("%d ", temp->val);
        temp = temp->next;
    }
    printf("\n");
}

struct ListNode* deleteDuplicates(LinkedList* list) {
    struct ListNode *head = list->head;
    if (!head) return NULL;

    struct ListNode *current = head;
    while (current && current->next) {
        if (current->val == current->next->val) {
            struct ListNode *toDelete = current->next;
            current->next = toDelete->next;
            free(toDelete);
        } else {
            current = current->next;
        }
    }
    return head;
}

void InsertionSort(LinkedList* list) {
    struct ListNode *sorted = NULL; // Start with an empty sorted list
    struct ListNode *current = list->head; // Start with the first element of the unsorted list

    while (current != NULL) {
        struct ListNode **tail;
        struct ListNode *next = current->next; // Store the next node to process after this one

        // Find the location to insert the current node in the sorted list
        // This loop now checks for a greater value to sort in descending order
        for (tail = &sorted; *tail != NULL && (*tail)->val > current->val; tail = &((*tail)->next));

        // Insert the current node into the sorted list at the found location
        current->next = *tail;
        *tail = current;

        // Move to the next node in the unsorted list
        current = next;
    }

    // Update the original list head to the new sorted list's head
    list->head = sorted;
}

