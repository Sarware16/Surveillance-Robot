
#ifndef LINKED_LIST_H_
#define LINKED_LIST_H_

#include <stdbool.h>

struct ListNode {
    int val;
    int angle;  // Assuming this is for future use since it's not used in the provided functions.
    struct ListNode *next;
};

typedef struct {
    struct ListNode* head;
} LinkedList;

void Insert(LinkedList* list, int x, int y);
void Print(LinkedList* list);
struct ListNode* deleteDuplicates(LinkedList* list);
void InsertionSort(LinkedList* list);

#endif /* LINKED_LIST_H_ */
