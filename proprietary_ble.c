
/*
Problem Description:

The proprietary wireless protocol utilized by our proprietary medical device
implements a pairing mode to discover and bind to nearby devices. Each time an
available device is observed, the wireless stack generates a pairing
advertisement event with the following parameters:

typedef struct {
  uint32_t device_id;
  uint8_t device_name[16];
  uint8_t device_data[64];
  uint32_t rf_address;
  uint8_t rssi;
} pair_adv_data_t;

A device is uniquely identified by its device_id. The device_id and all other
members of the pair_adv_data_t struct, with the exception of rssi, will never
change for a given device. Generally each available device will be observed 1-2
times per second.

1. Keep track of the advertising data (members of pair_adv_data_t) and time
since last observation for the 32 most recently observed devices. Use the
function systime_ms_get() to obtain the current system time in milliseconds.

2. Implement a function which prints a table containing device_id, device_name,
rssi, and time since last observation for each of the 32 most recently observed
devices. Order the table entries by rssi.

You may use any C standard library functions and the problem should typically
take one hour.

Please also describe any design tradeoffs you had in your submission.

*/

/* 
Zachary Bernal 2/10/2020
for [REDACTED] interview

This code runs on a linux system with:
	gcc -g -Wall -lm -o proprietary_ble proprietary_ble.c
	chmod +x proprietary_ble
	./proprietary_ble

My goals for this code:

1. keep in order of discovery time; for this we can use a queue. this allows us to drop stale data to keep only 32 in memory.

2. delete duplicates as they are discovered, to keep frequently-advertising devices from pushing out other devices in the queue. We are going to find more duplicate advertisements than new ones, especially in the average case where we have the same 20 devices advertising in a building.  The worst case is being at a trade show with 100's of devices advertising, so I don't want more frequent devices to drown out less-frequent devices. 
Unfortunately I cannot make discovery (device queue insertion) O(1) if I have to check for duplicates. It must be O(n) in order to find the duplicate so I can remove / reuse the device queue node. If we allow duplicate advertisements to push out the more quiet devices, we could speed up discovery to make it an O(1) operation. But that wouldn't be as fun to code.

3. Just use an O(n^2) sort at the end. Since I'm dealing with such a small number of elements the overhead from a O(nlogn) sorting algorithm isn't worth it. I don't expect printing to happen frequently (probably every ~10 seconds) if this function is meant for a user to look at manually. 

At first I considered using a circular buffer to maintain the queue, which has the added benefit of giving me static memory allocation for free. However, in order to reorder the buffer after detecting a duplicate device I would be constantly copying buffer elements of 120 bytes (sizeof(device_t)), which would be a lot of CPU crunchtime that I can avoid with other methods.

I decided to go with a doubly linked list to implement the device queue, for O(1) push and pop from both ends.

I decided to use a static memory pool since all the device objects are the same size, though for this exercise I probably have gotten away with using malloc() and free(). In an embedded system memory pools are desirable because:
- You don't have to deal with memory fragmentation caused by other code competing for memory
- You can put a hard limit on the amount of memory a specific module uses, and profile its memory usage
- You can give the linker and your build system prior knowledge of how much memory firmware will use by placing the pool buffer in .data or .bss

I really didn't need to do this much, but it was fun and I'm procrastinating talking to more recruiters. You can disable my nonsense by defining USE_FIXED_POOL to 0.

None of this is thread-safe, but I don't feel like having *that* much fun with this. Mutexes could be used to wrap the queue and the memory pool if needed.

*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <inttypes.h>
#include <math.h>
#include <time.h>

#define USE_FIXED_POOL 1

typedef struct {
  uint32_t device_id;
  uint8_t device_name[16];
  uint8_t device_data[64];
  uint32_t rf_address;
  uint8_t rssi;
} pair_adv_data_t;

typedef struct device {
	// Data returned from advertising function
	pair_adv_data_t adv;
	unsigned long long discovery_time;
	
	// Queue implemented with doubly linked list
	struct device *next;
	struct device *prev;
} device_t;

/*
 * ==========================
 * Mock systime_ms_get()
 * ==========================
 */
// Return value must be at least 64-bit to contain a unix timestamp (32 bits) with millisecond precision (needs at least 10 more bits)
unsigned long long systime_ms_get() {
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
	// Whole seconds and milliseconds are stored separately. Add both fields to obtain a timestamp in ms.
	return (unsigned long long)spec.tv_sec * 1000 + round(spec.tv_nsec / 1000000.0);
}

/*
 * ==========================
 * Device memory pool
 * ==========================
 */

#if USE_FIXED_POOL

typedef struct blockheader {
	// Next node in the stack
	struct blockheader *nextfree;
} blockheader_t;

typedef struct fixedpool {
	// Top of the stack
	struct blockheader *nextfree;
	size_t blocksize;
	uint32_t blockcount;
} fixedpool_t;

uint8_t device_pool[ sizeof(fixedpool_t) + (32 * (sizeof(blockheader_t) + sizeof(device_t))) ];

#define GET_BLOCK_FROM_MEM(mem) ( (blockheader_t *)((void *)mem - sizeof(blockheader_t)) )
#define GET_MEM_FROM_BLOCK(block) ( (void *)block + sizeof(blockheader_t) )

void pool_init(uint8_t *pool, size_t blocksize, uint32_t blockcount) {
	int i;
	fixedpool_t *header = (fixedpool_t *)pool;
	header->blocksize = blocksize;
	header->blockcount = blockcount;
	
	blockheader_t *prev = NULL;
	blockheader_t *cur = NULL;
	// Walk backwards to build the stack, so that it allocates in memory order.
	for (i = header->blockcount-1; i >=0; i--) {
		cur = (blockheader_t *)&pool[ sizeof(fixedpool_t) + (i * (sizeof(blockheader_t) + header->blocksize))];
		cur->nextfree = prev;
		prev = cur;
	}
	header->nextfree = cur;
}

void pool_destroy(uint8_t *pool) {
	memset(pool, 0, sizeof(fixedpool_t));
}

void * pool_alloc(uint8_t *pool, size_t size) {
	fixedpool_t *header = (fixedpool_t *)pool;
	blockheader_t *block = header->nextfree;
	if (block != NULL) {
		header->nextfree = block->nextfree;
		block->nextfree = NULL;
		return GET_MEM_FROM_BLOCK(block);
	} else {
		return NULL;
	}
}

void pool_free(uint8_t *pool, void *ptr) {
	fixedpool_t *header = (fixedpool_t *)pool;
	blockheader_t *block = GET_BLOCK_FROM_MEM(ptr);
	
	// TODO: check that ptr is in bounds
	if (ptr == NULL) {
		printf("WARNING: null free!\n");
		return;
	}
	
	if (block->nextfree != NULL){
		printf("WARNING: double free! ptr: %p nextfree: %p\n", ptr, block->nextfree);
	}
	
	block->nextfree = header->nextfree;
	header->nextfree = block;
	//printf("Freeing mem: %p block: %p\n", ptr, block);
}

void pool_print(uint8_t *pool) {
	int i;
	fixedpool_t *header = (fixedpool_t *)pool;
	printf("Fixed pool: %p size: %lu count: %u nextfree: %p\n", pool, header->blocksize, header->blockcount, header->nextfree);
	
	blockheader_t *cur = NULL;
	void *mem = NULL;
	for (i = 0; i < header->blockcount; i++) {
		cur = (blockheader_t *)&pool[ sizeof(fixedpool_t) + (i * (sizeof(blockheader_t) + header->blocksize))];
		mem = GET_MEM_FROM_BLOCK(cur);
		printf("block[%d]: %p mem: %p nextfree: %p\n", i, cur, mem, cur->nextfree);
	}
	
	// walk through and tell me how many nodes we hit
	int freecount = 0;
	for (cur = header->nextfree; cur != NULL; cur = cur->nextfree) freecount++;
	printf("Free count by walking: %d\n", freecount);
}


#else //USE_FIXED_POOL

uint8_t device_pool[4];

void pool_init(uint8_t *pool, size_t blocksize, uint32_t blockcount) {
}

void pool_destroy(uint8_t *pool) {
}

void * pool_alloc(uint8_t *pool, size_t size) {
	return malloc(size);
}

void pool_free(uint8_t *pool, void *ptr) {
	free(ptr);
}

void pool_print(uint8_t *pool) {
}

#endif


/*
 * ==========================
 * Device queue
 * ==========================
 */
int device_count = 0;
device_t *head = NULL;
device_t *tail = NULL;

/*
 * Find a duplicate device in the queue.
 * Returns: -1 for no duplicate, or pointer to duplicate
 */
device_t * find_duplicate(pair_adv_data_t *data) {
	device_t *cur = head;
	for (cur = head; cur != NULL; cur = cur->next) {
		// device_id is enough to uniquely identify a device
		if (cur->adv.device_id == data->device_id) {
			break;
		}
	}
	return cur;
}

void queue_remove(device_t *node) {
	if (node != NULL) {
		if (head == node) head = node->next;
		if (tail == node) tail = node->prev;
		if (node->prev != NULL) node->prev->next = node->next;
		if (node->next != NULL) node->next->prev = node->prev;
		node->prev = NULL;
		node->next = NULL;
		--device_count;
	}
}

void queue_push(device_t *node) {
	if (node != NULL) {
		node->prev = NULL;
		node->next = head;
		if (head != NULL) head->prev = node;
		head = node;
		if (tail == NULL) tail = node;
		++device_count;
	}
}

device_t * queue_pop(void) {
	device_t *node = tail;
	if (node != NULL) {
		tail = node->prev;
		if (tail != NULL) tail->next = NULL;
		if (head == node) head = NULL;
		--device_count;
	}
	return node;
}

void queue_clear(void) {
	while (head != NULL) {
		pool_free(device_pool, queue_pop());
	}
}

/*
 * ==========================
 * Device discovery and printing
 * ==========================
 */
void on_discovery(pair_adv_data_t *data) {
	unsigned long long timestamp = systime_ms_get();
	
	device_t *dupe = find_duplicate(data); // O(n)

	if (dupe != NULL){
		queue_remove(dupe); 
		queue_push(dupe); 
		dupe->adv.rssi = data->rssi;
		dupe->discovery_time = timestamp;
	}
	else
	{
		device_t *new;
		
		// This protects against an edge case 
		// where we somehow get more than 32 devices
		// in the list. Shouldn't happen unless there's a bug.
		while (device_count > 32) {
			printf("WARNING: large device_count %d\n", device_count);
			pool_free(device_pool, queue_pop());
		}
		if (device_count == 32) {
			// reuse oldest slot instead of reallocating
			device_t *old = queue_pop();
			new = old;
		}
		else {
			new = pool_alloc(device_pool, sizeof(device_t));
		}
		memcpy(new, data, sizeof(pair_adv_data_t));
		new->discovery_time = timestamp;
		queue_push(new);
	}
	
}

void print_queue_by_time(void) {
	device_t *cur;
	printf("Devices ordered by time (queue ordering):\n");
	for (cur = head; cur != NULL; cur = cur->next) {
		printf("time: %llu\tdev: %d\trssi: %d\n", 
				cur->discovery_time, 
				cur->adv.device_id,
				cur->adv.rssi);
	}
}

void print_queue_by_rssi(void) {
	// Sort by RSSI using insertion sort, which is good enough for small # of elements O(n^2)
	device_t *sorted[32] = {0};
	int next_i = 0;
	int j = 0;
	device_t *to_insert;
	
	for (to_insert = head; to_insert != NULL; to_insert = to_insert->next) {
		if (next_i >= 32) {
			printf("WARNING: large device_count %d\n", device_count);
			break;
		}
		sorted[next_i] = to_insert;
		j = next_i;
		while(j > 0 && sorted[j]->adv.rssi > sorted[j-1]->adv.rssi) {
			device_t *temp = sorted[j];
			sorted[j] = sorted[j-1];
			sorted[j-1] = temp;
			j--;
		}
		next_i++;
	}
	printf("Devices ordered by RSSI (descending):\n");
	for (j = 0; j < 32; j++) {
		printf("time: %llu\tdev: %d\trssi: %d\n", 
				sorted[j]->discovery_time, 
				sorted[j]->adv.device_id,
				sorted[j]->adv.rssi);
	}
}

/*
 * ==========================
 * Tests
 * ==========================
 */

void test_time(void) {
	unsigned long long timestamp;
	timestamp = systime_ms_get();
	printf("======== test_time ========\n");
	printf("Timestamp: %llu\n", timestamp);
}


#if USE_FIXED_POOL
#define TEST_POOL_SIZE 4
void test_pool(void) {
	int i;
	device_t *pointers[TEST_POOL_SIZE];
	
	printf("======== test_pool ========\n");
	
	pool_init(device_pool, sizeof(device_t), TEST_POOL_SIZE);
	pool_print(device_pool);
	
	for (i = 0; i < TEST_POOL_SIZE; i++) {
		pointers[i] = pool_alloc(device_pool, sizeof(device_t));
	}
	pool_print(device_pool);
	for (i = 0; i < TEST_POOL_SIZE; i++) {
		pool_free(device_pool, pointers[i]);
	}
	pool_print(device_pool);
	pool_destroy(device_pool);
}
#else
void test_pool(void) {
}
#endif

// Test that the queue can fill up and pop old devices off the end
void test_queue_fill(void) {
	int i;
	pair_adv_data_t cur = {0};
	uint8_t rssi = 1;
	printf("======== test_queue_fill ========\n");
	for (i = 1; i <= 64; i++) {
		cur.rssi = rssi;
		cur.device_id = i;
		// unimportant info for this test run
		/*
		sprintf((char *)cur.device_name, "proprietary_%02d", i);
		sprintf((char *)cur.device_data, "data_%02d", i);
		cur.rf_address = i;
		*/
		
		on_discovery(&cur);
		// wait a bit to get a new timestamp
		usleep(5 *1000);
		// scramble RSSI 
		rssi += 21;
	}
	print_queue_by_time();
	queue_clear();
}

// Reinstert the same 5 devices over and over again
void test_duplicates(void) {
	int i, j;
	pair_adv_data_t cur = {0};
	uint8_t rssi = 1;
	printf("======== test_duplicates ========\n");
	for (j = 1; j <= 3; j++) {
		printf("Round %d\n", j);
		for (i = 1; i <= 5; i++) {
			cur.rssi = rssi;
			cur.device_id = i;			
			on_discovery(&cur);
			// wait a bit to get a new timestamp
			usleep(5 *1000);
			
			// scramble RSSI 
			rssi += 21;
		}
		print_queue_by_time();
	}
	queue_clear();
}

// Make sure recurring devices don't drown out all others
void test_duplicates_and_uniques(void) {
	int i, j;
	int udc = 6; // unique device counter
	pair_adv_data_t cur = {0};
	uint8_t rssi = 1;
	printf("======== test_duplicates_and_uniques ========\n");
	
	// unique devices
	for (i = 1; i <= 32; i++) {
		cur.rssi = rssi;
		cur.device_id = udc++;			
		on_discovery(&cur);
		// wait a bit to get a new timestamp
		usleep(5 *1000);
		
		// scramble RSSI 
		rssi += 21;
	}
	// recurring devices; they should not push out old devices
	for (j = 1; j <= 7; j++) {
		for (i = 1; i <= 5; i++) {
			cur.rssi = rssi;
			cur.device_id = i;			
			on_discovery(&cur);
			// wait a bit to get a new timestamp
			usleep(5 *1000);
			
			// scramble RSSI 
			rssi += 21;
		}
	}
	print_queue_by_time();
	print_queue_by_rssi();
	queue_clear();
}

int main(int argc, char**argv) {
	printf("Proprietary BLE pairing test\n");
	test_pool();
	pool_init(device_pool, sizeof(device_t), 32);
	test_time();
	test_queue_fill();
	//pool_print(device_pool);
	test_duplicates();
	//pool_print(device_pool);
	test_duplicates_and_uniques();
	//pool_print(device_pool);
	return 0;
}

