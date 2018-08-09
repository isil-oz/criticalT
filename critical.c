/*
  trace.c

  Copyright 1998-2009 Virtutech AB
  
  The contents herein are Source Code which are a subset of Licensed
  Software pursuant to the terms of the Virtutech Simics Software
  License Agreement (the "Agreement"), and are being distributed under
  the Agreement.  You should have received a copy of the Agreement with
  this Licensed Software; if not, please contact Virtutech for a copy
  of the Agreement prior to using this Licensed Software.
  
  By using this Source Code, you agree to be bound by all of the terms
  of the Agreement, and use of this Source Code is subject to the terms
  the Agreement.
  
  This Source Code and any derivatives thereof are provided on an "as
  is" basis.  Virtutech makes no warranties with respect to the Source
  Code or any derivatives thereof and disclaims all implied warranties,
  including, without limitation, warranties of merchantability and
  fitness for a particular purpose and non-infringement.


*/
 
/*
  The trace facility will generate a sequence of function calls to a
  user-defined trace consumer. This is done for instruction fetches,
  data accesses, and exceptions. It is implemented by hooking in a
  memory-hierarchy into Simics and installing an hap handler that
  catches exceptions.

  The data passed is encoded in a trace_entry_t data structure and is
  on a "will execute" basis. The trace consumer is presented with a
  sequence of instructions that are about to execute, not that have
  executed. Thus, register contents are those that are in place prior
  to the instruction is executed. Various conditions will
  stop the instruction from executing, primarly exceptions -
  these generate new trace entries. Memory instructions that are
  executed will also generate memory access trace entries. Memory
  instructions that cause a fault will generate a memory access trace
  entry upon correct execution, typically after one or more fault
  handlers have executed.
*/

/*
  <add id="simics generic module short">
  <name>Trace</name><ndx>trace</ndx>
  <ndx>memory traces</ndx><ndx>instruction traces</ndx>
  This module provides an easy way of generating traces from Simics.
  Actions traced are executed instructions, memory accesses and,
  occurred exceptions. Traces will by default be printed as text to the
  terminal but can also be directed to a file in which case a binary
  format is available as well.

  The data presented is on a "will execute" basis. The trace will
  contain a sequence of instructions that are about to execute, not
  that have executed. Thus, register contents are those that are in
  place prior to the instruction is executed. Various conditions will
  stop the instruction from executing, primarly exceptions &mdash;
  these generate new trace entries. Memory instructions that are
  executed will generate memory access trace entries. Memory
  instructions that cause a fault will generate a memory access trace
  entry upon correct execution, typically after one or more fault
  handlers have executed.

  See the <i>trace</i> section of the <i>Commands</i> chapter in the
  <i>Simics Reference Manual</i> for the available trace
  commands. Also look at the documentation for the class base_trace
  that have some attributes that can be set to control what will be
  traced.

  </add> */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <stdlib.h>
 
#include <simics/api.h>
#include <simics/alloc.h>
#include <simics/utils.h>
#include <simics/arch/sparc.h>
#include <simics/arch/x86.h>

#if defined(HAVE_LIBZ)
 #include <zlib.h>
 #define GZ_FILE(bt) ((bt)->gz_file)
 #define GZ(x)       (x)
#else
 #define GZ_FILE(bt) NULL
 #define GZ(x)
#endif

#define MAX_THREADS 8
#define PWEIGHT 0.8
#define REMOTE_THRESHOLD 100
//#define MEMORY_MAP_THRESHOLD 100000

//#define THREAD_LEVEL //core level vs thread level
//#define FAN //fan info bilgisi tutulsun
//#define REDUNDANCY //redundant thread var mı?
#define REGION //redundant region var mı?

//#define FAULT_INJECTION 

#define INT_TYPE 0
#define DOUBLE_TYPE 1
#define LONG_LONG_TYPE 2

#include "trace.h"
#include "strmap.h"

//threadlerin fan-in, fan-out bilgileri
typedef struct {
        double direct_alu_array[MAX_THREADS][MAX_THREADS];
        double direct_reg_array[MAX_THREADS][MAX_THREADS];
        double direct_mem_array[MAX_THREADS][MAX_THREADS];
        double direct_count[MAX_THREADS][MAX_THREADS];

        double indirect_alu_array[MAX_THREADS][MAX_THREADS];
        double indirect_reg_array[MAX_THREADS][MAX_THREADS];
        double indirect_mem_array[MAX_THREADS][MAX_THREADS];
        double indirect_count[MAX_THREADS][MAX_THREADS];
}Fan_info;

//store operasyonunda o anki bilgileri saklamak için
typedef struct {
        int thread_no;
        double alu_tvf;
        double reg_tvf;
        double mem_tvf;

        double alu_local_interval;
        double reg_local_interval;
        double mem_local_interval;

        double alu_lvf;
        double reg_lvf;
        double mem_lvf;

        long long instr_num[MAX_THREADS];
        Fan_info *fan_info;
}Store_info;

typedef struct {
	int thread_num;
	int thread_index;
	int core_map;
	long long instr_num;
	long long instr_count;
	long long reg_live_area;
	long long mem_live_area;
	long long remote_count;
	//for each register and memory location
	StrMap *register_vul;
	StrMap *memory_vul;
        //self local VF values
        double reg_self_vul;
        double mem_self_vul;
        double alu_self_vul;
        //local VF values
        double reg_vul;
	double mem_vul;
	double alu_vul;
        //VF values on the last remote write
        double reg_last_remote;
        double mem_last_remote;
        double alu_last_remote;
	//remote VF values for each dependent thread
        StrMap *remote_reg_vul;
	StrMap *remote_mem_vul;
	StrMap *remote_alu_vul;
	StrMap *remote_count_vul;
	//remote VF values
	double rem_reg_vul;
	double rem_mem_vul;
	double rem_alu_vul;
        //load operations by the thread itself
	StrMap *memory_load;
}thread_t;

typedef struct {
	int active;
}core_t;

/* Cached information about a processor. */
typedef struct {
        unsigned va_digits;
        unsigned pa_digits;
        conf_object_t *cpu;
        char name[10];
        processor_info_interface_t *info_iface;
        exception_interface_t *exception_iface;
} cpu_cache_t;

typedef struct {
        generic_address_t start;
        generic_address_t end;
} interval_t;
typedef VECT(interval_t) interval_list_t;

enum { PHYSICAL, VIRTUAL, NUM_ADDRESS_TYPES };

   
typedef struct base_trace {
        log_object_t log;

        /*
         * Scratch area used for communicating trace event data. This is
         * maintained on a per processor basis.
         */
        trace_entry_t current_entry;
        trace_entry_t last_entry;

        /* Trace to file if file_name is non-NULL. */
        char *file_name;
        int warn_on_existing_file;
        FILE *file;
#if defined(HAVE_LIBZ)
        gzFile gz_file;
#endif


        /* Count the events of each type. */
	core_t cores[32];
	thread_t threads[MAX_THREADS];

	thread_t rr_thread;//redundant region data

	int cpu0_active;
	int cpu1_active;
	int cpu2_active;
	int cpu3_active;
	int cpu4_active;
	int cpu5_active;
	int cpu6_active;
	int cpu7_active;
        int cpu8_active;
	int cpu9_active;
	int cpu10_active;
	int cpu11_active;
	int cpu12_active;
	int cpu13_active;
	int cpu14_active;
	int cpu15_active;
	int cpu16_active;
	int cpu17_active;
	int cpu18_active;
	int cpu19_active;
	int cpu20_active;
	int cpu21_active;
	int cpu22_active;
	int cpu23_active;
        int cpu24_active;
	int cpu25_active;
	int cpu26_active;
	int cpu27_active;
	int cpu28_active;
	int cpu29_active;
	int cpu30_active;
	int cpu31_active;

	int partial_result;
	int region;

        int redundant;

	int redundant_thread;//hangi threadi partial redundant yapıcam
	int redundant_region;//partial thread simdi redundant region icinde mi

	//fault injection related variables (factors)
	long long fault_injection_instruction;
	int fault_injection_core;
	
	hap_type_t fault_handle;

        uint64 exec_count;
        uint64 data_count;
        uint64 exc_count;

	char read_value[100];
	char write_value[100];
	
        /* 0 for text, 1 for raw */ 
        int trace_format;   

        conf_object_t *consumer;
        trace_consume_interface_t consume_iface;

        /* Cached processor info. */
        cpu_cache_t *cpu;

        /* "Processor info" for non-cpu devices. */
        cpu_cache_t device_cpu;

        /* True if we are currently hooked into the memory hierarchy, false
           otherwise. */
        int memhier_hook;

        int trace_enabled;

        int trace_exceptions;
        int trace_instructions;
        int trace_data;
        int filter_duplicates;

        int print_physical_address;
        int print_virtual_address;
        int print_linear_address;
        int print_access_type;
        int print_memory_type;
        int print_data;

        /* Lists of intervals to trace data accesses for. _stc_ are the same
           lists, butrounded outwards to DSTC block size. */
        interval_list_t data_interval[NUM_ADDRESS_TYPES];
        interval_list_t data_stc_interval[NUM_ADDRESS_TYPES];

        cycles_t last_timestamp;

	StrMap *memory_store;
	StrMap *thread_no_map;

	/*node *front;    // front pointer in queue
    	node *rear;     // rear pointer in queue
	  //  rear=front=NULL;
	int index_array[MEMORY_MAP_THRESHOLD];
	int index_counter;
	int oldest_index;*/
        //threadlerin fan-in, fan-out durumlarını saklar
        Fan_info *fan;

        /*
         * Function pointer to the trace consumer (if there is one). In future
         * we may want to support multiple trace consumers, especially if we do
         * proper statistical sampling of various characteristics, in which
         * case to reduce unwanted covariance we want separate sampling, with
         * this facility capable of handling overlaps.
         */
        void (*trace_consume)(struct base_trace *bt, trace_entry_t *);

#if defined(TRACE_STATS)
        uint64 instruction_records;
        uint64 data_records;
        uint64 other_records;
#endif   /* TRACE_STATS */
} base_trace_t;


typedef struct trace_mem_hier_object {
        conf_object_t obj;
        base_trace_t *bt;

        /* For forwarding requests. */
        conf_object_t *timing_model;
        timing_model_interface_t timing_iface;
        conf_object_t *snoop_device;
        timing_model_interface_t snoop_iface;
} trace_mem_hier_object_t;

static const char *const read_or_write_str[] = { "Read ", "Write"};

const char *const seg_regs[] = {"es", "cs", "ss", "ds", "fs", "gs", 0};

static void
external_tracer(base_trace_t *bt, trace_entry_t *ent)
{
        bt->consume_iface.consume(bt->consumer, ent);
}

//Store_info struct için stringe ve stringden çevirme fonksiyonları
void storeToString(Store_info *store, char info[])
{
    int i;

    sprintf(info, "%d+%f+%f+%f+%f+%f+%f+%f+%f+%f",
                        store->thread_no, store->alu_tvf, store->reg_tvf, store->mem_tvf,
                        store->alu_lvf, store->reg_lvf, store->mem_lvf,
                        store->alu_local_interval, store->reg_local_interval, store->mem_local_interval);

    for(i = 0; i < MAX_THREADS; i++)
        sprintf(info, "%s+%lld", info, store->instr_num[i]);

    #ifdef FAN
        int j;
        for(i = 0; i < MAX_THREADS; i++){
            for(j = 0; j < MAX_THREADS; j++){
                sprintf(info, "%s+%f+%f+%f+%f+%f+%f+%f+%f", info,
                        store->fan_info->direct_alu_array[i][j], store->fan_info->direct_reg_array[i][j], store->fan_info->direct_mem_array[i][j],
                        store->fan_info->direct_count[i][j],
                        store->fan_info->indirect_alu_array[i][j], store->fan_info->indirect_reg_array[i][j], store->fan_info->indirect_mem_array[i][j],
                        store->fan_info->indirect_count[i][j]);
            }
        }
    #endif
}

void stringToStore(char str[], Store_info *si)
{
    int i;
    char *saveptr1;
    si->thread_no = atoi(strtok_r(str, "+", &saveptr1));

    si->alu_tvf = atof(strtok_r(NULL, "+", &saveptr1));
    si->reg_tvf = atof(strtok_r(NULL, "+", &saveptr1));
    si->mem_tvf = atof(strtok_r(NULL, "+", &saveptr1));

    si->alu_lvf = atof(strtok_r(NULL, "+", &saveptr1));
    si->reg_lvf = atof(strtok_r(NULL, "+", &saveptr1));
    si->mem_lvf = atof(strtok_r(NULL, "+", &saveptr1));

    si->alu_local_interval = atof(strtok_r(NULL, "+", &saveptr1));
    si->reg_local_interval = atof(strtok_r(NULL, "+", &saveptr1));
    si->mem_local_interval = atof(strtok_r(NULL, "+", &saveptr1));

    for(i = 0; i < MAX_THREADS; i++)
        si->instr_num[i] = atoll(strtok_r(NULL, "+", &saveptr1));

    #ifdef FAN
        int j;
        for(i = 0; i < MAX_THREADS; i++){
            for(j = 0; j < MAX_THREADS; j++){
                si->fan_info->direct_alu_array[i][j] = atof(strtok_r(NULL, "+", &saveptr1));
                si->fan_info->direct_reg_array[i][j] = atof(strtok_r(NULL, "+", &saveptr1));
                si->fan_info->direct_mem_array[i][j] = atof(strtok_r(NULL, "+", &saveptr1));
                si->fan_info->direct_count[i][j] = atof(strtok_r(NULL, "+", &saveptr1));

                si->fan_info->indirect_alu_array[i][j] = atof(strtok_r(NULL, "+", &saveptr1));
                si->fan_info->indirect_reg_array[i][j] = atof(strtok_r(NULL, "+", &saveptr1));
                si->fan_info->indirect_mem_array[i][j] = atof(strtok_r(NULL, "+", &saveptr1));
                si->fan_info->indirect_count[i][j] = atof(strtok_r(NULL, "+", &saveptr1));
            }
        }
    #endif
}

//bir remote read olduğunda fan-in, fan-out updateleri
void handle_fan_info(Store_info *sinfo, Fan_info *finfo, int in)
{
    int out = sinfo->thread_no;

    finfo->direct_alu_array[out][in] += sinfo->alu_lvf;
    finfo->direct_reg_array[out][in] += sinfo->reg_lvf;
    finfo->direct_mem_array[out][in] += sinfo->mem_lvf;
    finfo->direct_count[out][in] += 1;

    //out threade in yapan diğer threadler (indirect etki)
    //direct etkisi belli bir sayıya geldikten sonra etki ettir
    if(finfo->direct_count[out][in] > REMOTE_THRESHOLD)
    {
        //printf("%d %d'ye yazdi %f\n", out, in, finfo->direct_count[out][in]);
        int j;
        for(j = 0; j < MAX_THREADS; j++)
        {
                //remote etki belli bir miktara geldikten sonra indirect etki ettir
                if(sinfo->fan_info->direct_count[j][out] > REMOTE_THRESHOLD || sinfo->fan_info->indirect_count[j][out] > REMOTE_THRESHOLD)
                {
                    //etki eden değerlerin en büyüğünü al
                    double new_value = (sinfo->fan_info->direct_alu_array[j][out] + sinfo->fan_info->indirect_alu_array[j][out]) * PWEIGHT;
                    if(new_value > finfo->indirect_alu_array[j][in])
                        finfo->indirect_alu_array[j][in] = new_value;
                    new_value = (sinfo->fan_info->direct_reg_array[j][out] + sinfo->fan_info->indirect_reg_array[j][out]) * PWEIGHT;
                    if(new_value > finfo->indirect_reg_array[j][in])
                        finfo->indirect_reg_array[j][in] = new_value;
                    new_value = (sinfo->fan_info->direct_mem_array[j][out] + sinfo->fan_info->indirect_mem_array[j][out]) * PWEIGHT;
                    if(new_value > finfo->indirect_mem_array[j][in])
                        finfo->indirect_mem_array[j][in] = new_value;
                    new_value = (sinfo->fan_info->direct_count[j][out] + sinfo->fan_info->indirect_count[j][out]) * PWEIGHT;
                    if(new_value > finfo->indirect_count[j][in])
                        finfo->indirect_count[j][in] = new_value;
                    //printf("%d %d'ye direct etki etmis %f, %d %d indirect etki edecek %f\n", j,out,sinfo->fan_info->direct_count[j][out], j,in,finfo->indirect_count[j][in]);
                }
        }
    }
}

int get_from_char_map(StrMap *map, char key[], void *value, int type)
{
        char buf[50];
        int result = strmap_get(map, key, buf, sizeof(buf));
        if(result != 0)
        {
                switch (type)
                {
                    case DOUBLE_TYPE:
                    {
                        double* pdouble = (double*)value;
                        *pdouble = atof(strtok(buf,""));
                        break;
                    }
                    case INT_TYPE:
                    {
                        int* pint = (int*)value;
                        *pint = atoi(strtok(buf,""));
                        break;
                    }
                    case LONG_LONG_TYPE:
                    {
                        long long* plong = (long long*)value;
                        *plong = atoll(strtok(buf,""));
                        break;
                    }
                    default:
                    {
                        printf("Undefined data type\n");
                        break;
                    }
                }
        }

        return result;
}

int get_from_int_map(StrMap *map, int key, void *value, int type)
{
        char keyx[50];
        sprintf(keyx, "%d", key);
        return get_from_char_map(map, keyx, value, type);
}

int put_to_char_map(StrMap *map, char key[], void *value, int type)
{
        char num[50];
        switch (type)
                {
                        case DOUBLE_TYPE:
                        {
                                double *pdouble = (double*)value;
                                sprintf(num, "%f", *pdouble);
                                break;
                        }
                        case INT_TYPE:
                        {
                                int *pint = (int*)value;
                                sprintf(num, "%d", *pint);
                                break;
                        }
                        case LONG_LONG_TYPE:
                        {
                                long long *plong = (long long*)value;
                                sprintf(num, "%lld", *plong);
                                break;
                        }
                        default:
                        {
                                printf("Undefined data type\n");
                                break;
                        }
                }
        return strmap_put(map, key, num);
}

int put_to_int_map(StrMap *map, int key, void *value, int type)
{
        char keyx[50];
        sprintf(keyx, "%d", key);
        return put_to_char_map(map, keyx, value, type);
}

thread_t *get_current_thread(base_trace_t *bt, int pid, int cpu_no)
{
        int mapid;
        int result = get_from_int_map(bt->thread_no_map, pid, &mapid, INT_TYPE);
        if(result != 0)
	{
                return &bt->threads[mapid];
	}else
	{
		int size = strmap_get_count(bt->thread_no_map);
                put_to_int_map(bt->thread_no_map, pid, &size, INT_TYPE);
                (&bt->threads[size])->thread_num = pid;
		(&bt->threads[size])->thread_index = size;
		(&bt->threads[size])->core_map = cpu_no;
                return &bt->threads[size];
	}
}

double calculate_lvf(double lvf, long long instr_num, int size)
{
    return lvf / ((double)instr_num * (double)size);
}

double calculate_rvf(double rvf, long long remote_count)
{
    return rvf / ((double)remote_count);
}

double calculate_tvf(double lvf, double rvf, double lweight, double rweight)
{
    return lweight * lvf + rweight * rvf;
}

static void
text_trace_data(base_trace_t *bt, trace_entry_t *ent, char *s)
{
	cpu_cache_t *cc;
	thread_t *temp;
	thread_t *temp2;
	core_t *core;
	signed cpu_no = ent->cpu_no;
	int thread_no = 0;

        int mem_type = 0;
	uint64 mem_adress;
        char mem_a[50];

        unsigned hit1 = 0;
	unsigned hit2 = 0;
	/* CPU number, addresses. */
   	cc = &bt->cpu[ent->cpu_no];
	s +=vtsprintf(s, "");

	#ifdef FAULT_INJECTION
		return;//no need to trace
	#endif

	core = &bt->cores[cpu_no];
	 
	if(core->active == 0)
	{
		return;
	}
        #ifdef THREAD_LEVEL
            int pid = core->active;
            temp = get_current_thread(bt, pid, cpu_no);
            thread_no = temp->thread_index;
        #else
            temp = &bt->threads[cpu_no];
            thread_no = cpu_no;
        #endif

	#ifdef REGION
		if((bt->redundant_region) == 1 && (cpu_no == bt->redundant_thread))
		{
			temp2 = &bt->rr_thread;
			if(temp2->thread_num != cpu_no)
			{
				//MM_FREE(&(bt->rr_thread));
				initialize_thread_data(&(bt->rr_thread), cpu_no);
			}
		}
	#endif
	//memory_store
        if (cpu_no == -1)
                cc = &bt->device_cpu;
        else
                cc = &bt->cpu[cpu_no];
        bt->data_count++;

        mem_type = ent->read_or_write;//0 read, 1 write
	mem_adress = ent->pa;//ent->value.data;
	hit1 = ent->L1_hit_or_miss;

        sprintf(mem_a, "%llx", mem_adress);

  	if(mem_type == 0) //load
	{
                char buf[50000];
                int result = 0;
                result = strmap_get(bt->memory_store, mem_a, buf, sizeof(buf));
		if(result != 0) //found, there is a store 
		{
                        Store_info *si = (Store_info *) malloc(sizeof(Store_info));
                        si->fan_info = (Fan_info *) malloc(sizeof(Fan_info));

                        stringToStore(buf, si);

                        int store_thread = si->thread_no;
                        long long store_inst = si->instr_num[thread_no];

                        if(hit1 == 1 || hit2 == 1)//hit, cache vulnerability local vulnerability
			{
				double mem_value = 0;
                                long long last_load;
                                //store sonrasında load var mı bakalım, varsa ondan sonrası için vulnerability hesaplayalım, oncesi için zaten hesaplanmıştır
				// 1)store X,  2)load X (1-2 arası hesapla),  3)load X (2-3 arası hesapla)
                                result = get_from_char_map(temp->memory_load, mem_a, &last_load, LONG_LONG_TYPE);
                                if((result != 0) && (last_load > store_inst))
				{
                                    store_inst = last_load;
				}

				mem_value = temp->instr_num - store_inst;
                                //bu memorynin önceki vul değerini alalım
				double old_value = 0;
                                get_from_char_map(temp->memory_vul, mem_a, &old_value, DOUBLE_TYPE);
                                if(mem_value != 0)
                                {
					temp->mem_live_area++;
					temp->mem_vul = temp->mem_vul + mem_value;
                                        mem_value = mem_value + old_value;
                                        put_to_char_map(temp->memory_vul, mem_a, &mem_value, DOUBLE_TYPE);
                                }
				//hangi instructionda load ettik yazalım
                                put_to_char_map(temp->memory_load, mem_a, &(temp->instr_num), LONG_LONG_TYPE);

				
				if(temp2)
				{
					mem_value = 0;
					result = get_from_char_map(temp2->memory_load, mem_a, &last_load, LONG_LONG_TYPE);
		                        if((result != 0) && (last_load > store_inst))
					{
		                            store_inst = last_load;
					}

					mem_value = temp2->instr_num - store_inst;
		                        //bu memorynin önceki vul değerini alalım
					double old_value = 0;
		                        get_from_char_map(temp2->memory_vul, mem_a, &old_value, DOUBLE_TYPE);
		                        if(mem_value != 0)
		                        {
						temp2->mem_live_area++;
						temp2->mem_vul = temp2->mem_vul + mem_value;
		                                mem_value = mem_value + old_value;
		                                put_to_char_map(temp2->memory_vul, mem_a, &mem_value, DOUBLE_TYPE);
		                        }
					//hangi instructionda load ettik yazalım
		                        put_to_char_map(temp2->memory_load, mem_a, &(temp2->instr_num), LONG_LONG_TYPE);
				}
				
                        }
			if(store_thread != thread_no)//another thread store, remote vulnerability
			{
                                double remote_alu = si->alu_tvf;
                                double remote_reg = si->reg_tvf;
                                double remote_mem = si->mem_tvf;

                                //remote vulnerability
				temp->rem_alu_vul = temp->rem_alu_vul + remote_alu;
               			temp->rem_reg_vul = temp->rem_reg_vul + remote_reg;
                                temp->rem_mem_vul = temp->rem_mem_vul + remote_mem;
				
				temp->remote_count++;

				//which thread stored info
				remote_vulnerability(temp, store_thread, remote_alu, remote_reg, remote_mem);

                                /*hangi threadden remote data geldiyse, o threadin LVF değerlerini sıfır yapalım
                                  (23.12.2010 Mahmut hocanın önerisi: RVF'den çok küçük LVF almak için, LVF zaten RVF'i hesaplarken kullanıldı mantığıyla)
                                  */

                                thread_t* rt = &bt->threads[store_thread];

                                //0 olmayan bir önceki değeri tutalim ki 0 olursa bunu kullanalım
                                if(rt->alu_self_vul != 0)
                                    rt->alu_last_remote = rt->alu_self_vul;
                                if(rt->mem_self_vul != 0)
                                    rt->mem_last_remote = rt->mem_self_vul;
                                if(rt->reg_self_vul != 0)
                                    rt->reg_last_remote = rt->reg_self_vul;

                                rt->alu_self_vul = si->alu_local_interval;
                                rt->mem_self_vul = si->mem_local_interval;
                                rt->reg_self_vul = si->reg_local_interval;

                                //handle fan
                                #ifdef FAN
                                    handle_fan_info(si, bt->fan, thread_no);
                                #endif
                        }
                        MM_FREE(si->fan_info);
                        MM_FREE(si);
		}
	}
	else if(mem_type == 1) //store
	{
                double local_alu = 0, local_reg = 0, local_mem = 0;
                double remote_alu = 0, remote_reg = 0, remote_mem = 0;
                double alu = 0, reg = 0, mem = 0;
                long long remote_count = 1;
		int reg_size = 0, mem_size = 0;
                if(temp->remote_count != 0) remote_count = temp->remote_count;
		//if(temp->reg_live_area != 0) reg_area = temp->reg_live_area; 
		//if(temp->mem_live_area != 0) mem_area = temp->mem_live_area; 
		double localweight = 0.5;
		double remoteweight = 0.5;
                //etki edecek remote vul degerini hesaplıyorum
		if(temp->instr_num != 0)
		{
                        if(remote_count == 0)
                        {
				remote_count = 1;
				localweight = 1;
				//remoteweight = 0;
			}
			
			if(temp2)
			{
				reg_size = strmap_get_count(temp2->register_vul);
				if(reg_size == 0) reg_size = 1;
                        	mem_size = strmap_get_count(temp2->memory_vul);
				if(mem_size == 0) mem_size = 1;

                        	local_reg = calculate_lvf(temp2->reg_vul, temp2->instr_num, reg_size);
                        	local_mem = calculate_lvf(temp2->mem_vul, temp2->instr_num, mem_size);
                        	local_alu = calculate_lvf(temp2->alu_vul, temp2->instr_num, 1);
			}
			else
			{
                        	reg_size = strmap_get_count(temp->register_vul);
				if(reg_size == 0) reg_size = 1;
                        	mem_size = strmap_get_count(temp->memory_vul);
				if(mem_size == 0) mem_size = 1;

                        	local_reg = calculate_lvf(temp->reg_vul, temp->instr_num, reg_size);
                        	local_mem = calculate_lvf(temp->mem_vul, temp->instr_num, mem_size);
                        	local_alu = calculate_lvf(temp->alu_vul, temp->instr_num, 1);
			}

                        remote_reg = calculate_rvf(temp->rem_reg_vul, remote_count);
                        remote_mem = calculate_rvf(temp->rem_mem_vul, remote_count);
                        remote_alu = calculate_rvf(temp->rem_alu_vul, remote_count);

                        #ifdef REDUNDANCY
                        if(thread_no == bt->redundant)
                        {
                            local_reg = local_reg * local_reg;
                            local_mem = local_mem * local_mem;
                            local_alu = local_alu * local_alu;
                        }
                        #endif
			
			if(temp2)
			{
				local_reg = local_reg * local_reg;
                            	local_mem = local_mem * local_mem;
                            	local_alu = local_alu * local_alu;
                        }
			
			
                        reg = calculate_tvf(local_reg, remote_reg, localweight, remoteweight);
                        mem = calculate_tvf(local_mem, remote_mem, localweight, remoteweight);
                        alu = calculate_tvf(local_alu, remote_alu, localweight, remoteweight);
                }
                //store operation sırasında ne tutmak istiyorsak bu veri yapısında
                Store_info *si = (Store_info *) malloc(sizeof(Store_info));
                si->fan_info = bt->fan;
                si->thread_no = thread_no;

                si->alu_tvf = alu;
                si->mem_tvf = mem;
                si->reg_tvf = reg;
                //remote olarak okunduğu zaman çıkarabilmek için o andaki local VF değerlerini de biryerde saklayalım
                si->alu_local_interval = temp->alu_vul;
                si->mem_local_interval = temp->mem_vul;
                si->reg_local_interval = temp->reg_vul;

                //printf("Store %s thread %d alu local %f\n", mem_a, thread_no, si->alu_local_interval);
                #ifdef FAN
                    si->alu_lvf = local_alu;
                    si->mem_lvf = local_mem;
                    si->reg_lvf = local_reg;
                #endif

                for(int i = 0; i < MAX_THREADS;i++)
		{
                    si->instr_num[i] = bt->threads[i].instr_num;
                }

                char info[50000];

                storeToString(si, info);

                MM_FREE(si);
		/*int xx = */strmap_put(bt->memory_store, mem_a, info);
		//handle_capacity(bt, mem_a);

	}		
}

void
remote_vulnerability(thread_t *bt, int store_thread, double alu_value, double reg_value, double mem_value)
{
        //which thread stored?
	StrMap *reg_vulnerability = bt->remote_reg_vul;
	StrMap *alu_vulnerability = bt->remote_alu_vul;
	StrMap *mem_vulnerability = bt->remote_mem_vul;
	StrMap *rem_count = bt->remote_count_vul;

        double remote_alu = alu_value, remote_reg = reg_value, remote_mem = mem_value;
        long long r_count = 1;

        int result_alu = get_from_int_map(alu_vulnerability, store_thread, &remote_alu, DOUBLE_TYPE);
        if(result_alu != 0)//there is vulnerability calculated
        {
                get_from_int_map(reg_vulnerability, store_thread, &remote_reg, DOUBLE_TYPE);
                get_from_int_map(mem_vulnerability, store_thread, &remote_mem, DOUBLE_TYPE);
                get_from_int_map(rem_count, store_thread, &r_count, LONG_LONG_TYPE);
                remote_alu += alu_value;
                remote_reg += reg_value;
                remote_mem += mem_value;
                r_count += 1;
        }

        put_to_int_map(reg_vulnerability, store_thread, &remote_reg, DOUBLE_TYPE);
        put_to_int_map(alu_vulnerability, store_thread, &remote_alu, DOUBLE_TYPE);
        put_to_int_map(mem_vulnerability, store_thread, &remote_mem, DOUBLE_TYPE);
        put_to_int_map(rem_count, store_thread, &r_count, LONG_LONG_TYPE);
}

double
reg_vulnerability(StrMap *vulnerability, long long current_inst_num, char *source, int read, char *s)
{
	char buf[1000];
	char vul[1000];
        char *saveptr1;
	
        int result = strmap_get(vulnerability, source, buf, sizeof(buf));
        double vul_value = 0;
        double ret_value = 0;

	if(result != 0)//there is vulnerability calculated
	{
		long long instr_num = atoll(strtok_r(buf, "+", &saveptr1));
                double old_value = atof(strtok_r(NULL, "+", &saveptr1));
      		if(read == 1)//read 
      		{
                        double new_vul = current_inst_num - instr_num;
			vul_value = old_value + new_vul;
			ret_value = new_vul;
      		}else//write
      		{
      			vul_value = old_value;	
		}
	}
	
        sprintf(vul, "%lld+%f", current_inst_num, vul_value);

        strmap_put(vulnerability, source, vul);
        return ret_value;
}

static void
text_trace_exception(base_trace_t *bt, trace_entry_t *ent, char *s)
{
	
	SIM_clear_exception();
	s +=vtsprintf(s, "");
	return;

        cpu_cache_t *cc;

        if (ent->cpu_no == -1)
                cc = &bt->device_cpu;
        else
                cc = &bt->cpu[ent->cpu_no];
        bt->exc_count++;
	s += vtsprintf(s, "exce: [%9llu] %s", bt->exc_count, cc->name);
        s += vtsprintf(s, "exception %3d (%s)\n",
                       ent->value.exception,
                       cc->exception_iface->get_name(cc->cpu,
                                                     ent->value.exception));
        SIM_clear_exception();
}

#define EXEC_VA_PREFIX(ent) ((ent)->arch == TA_x86 ? "cs" : "v")



static void
text_trace_instruction(base_trace_t *bt, trace_entry_t *ent, char *s)
{
   	cpu_cache_t *cc;
	signed cpu_no = ent->cpu_no;
        cc = &bt->cpu[ent->cpu_no];

	thread_t *temp;
	thread_t *temp2;     

  	core_t *core;
	s +=vtsprintf(s, "");
	
	core = &bt->cores[cpu_no];
		
	if(core->active == 0)
	{
		return;
	}

	#ifdef THREAD_LEVEL
            int pid = core->active;
            temp = get_current_thread(bt, pid, cpu_no);
        #else
            temp = &bt->threads[cpu_no];
        #endif

	#ifdef REGION
		if((bt->redundant_region) == 1 && (cpu_no == bt->redundant_thread))
		{
			temp2 = &bt->rr_thread;
			if(temp2->thread_num != cpu_no)
			{
				//MM_FREE(&(bt->rr_thread));
				initialize_thread_data(&(bt->rr_thread), cpu_no);
			}
			++(temp2->instr_count);
			++(temp2->instr_num);
		}
	#endif
	++(temp->instr_count);
	
	/*if(temp->instr_count < 1000000000)
	{
		return;
	}*/
        /*if(temp->instr_count == THRESHOLD)
	{
		printf("Thread %d finished\n", cpu_no);
		return;
	}
        if(temp->instr_count > THRESHOLD)
	{
		return;
        }*/
	++(temp->instr_num);
	
	/*if(temp.instr_num % 1000000 == 0)
		printf("%d thread inst count %ld\n",cpu_no,temp.instr_num);*/

        bt->exec_count++;

	if(bt->partial_result == 1)
	{
		(bt->region)++;
		printf("writing partial\n");
		print_temp_results(bt);
		bt->partial_result = 0;
	}
	#ifdef FAULT_INJECTION
		if((cpu_no == bt->fault_injection_core) && (temp->instr_count == bt->fault_injection_instruction))
		{
			printf("Time for fault injection %ld on %d\n", bt->fault_injection_instruction, bt->fault_injection_core);
			if (SIM_hap_is_active(bt->fault_handle))
                        	SIM_c_hap_occurred(bt->fault_handle, bt, 0);
		}
		//printf("Not time for fault injection %d on %d\n", temp->instr_count, cpu_no);		
		return;//no need to trace
	#endif
        char read1[100];
	char write1[100];
	int isRead = 1;
	int isWrite = 1;
	int len = 0;
	//char *write;
	//char *read;
        /* Disassembly. */
        attr_value_t opcode = SIM_make_attr_data_adopt(ent->size, ent->value.text);
        tuple_int_string_t ret;
        int sub_operation;
        if (ent->arch == TA_ia64)
                sub_operation = (ent->va & 0xF);
        else
                sub_operation = 0;
        ret = cc->info_iface->disassemble(cc->cpu, ent->va, opcode, sub_operation);
	//SIM_attr_free(&opcode);
	//if (ret.string) 
	//	MM_FREE(ret.string);	
	
        
        if (ret.string) {
		char *saveptr1;
                char *instr = strtok_r(ret.string, " ", &saveptr1);
		
                //3 operand arithmetic instr.
                if ((strncmp (instr,"add",3) == 0) ||
                    (strncmp (instr,"fadd",4) == 0) ||
                	  (strncmp (instr,"sub",3) == 0) ||
                	  (strncmp (instr,"fsub",4) == 0) ||
		              (strncmp (instr,"mul",3) == 0) ||
		              (strncmp (instr,"fmul",4) == 0) ||
			      (strncmp (instr,"imul",4) == 0) ||
			      (strncmp (instr,"lea",3) == 0) ||
			      (strncmp (instr,"cmp",3) == 0) ||
		              (strncmp (instr,"and",3) == 0) ||
		              (strncmp (instr,"or",2) == 0) ||
		              (strncmp (instr,"xor",3) == 0) ||
		              (strncmp (instr,"shl",3) == 0) ||
		              (strncmp (instr,"shr",3) == 0) ||
		              (strncmp (instr,"test",4) == 0))
                {
                	char *write = strtok_r(NULL, ",", &saveptr1);
			if(write != NULL)
			{
				int len = strlen(write);
				if(len < 100)
				{
					strcpy(write1, write);
					isWrite = get_operand(write1, 1); 
				}
			}
                	char *read = strtok_r(NULL, ",", &saveptr1);
			if(read != NULL)  
			{			
				len = strlen(read);
				if(len < 100)
				{
					strcpy(read1, read);	
					isRead = get_operand(read1, 0);  
				}
			}
                	temp->alu_vul++;
			#ifdef REGION
				if(temp2)
				{
					temp2->alu_vul++;
				}
			#endif
                }
                else if((strncmp (instr,"mov",3) == 0) ||
			(strncmp (instr,"movzx",5) == 0))
                {
                	char *write = strtok_r(NULL, ",", &saveptr1);
			if(write != NULL)
			{
				len = strlen(write);
				if(len < 100)
				{
					strcpy(write1, write);
					isWrite = get_operand(write1, 1);  
				}
			}
                	char *read = strtok_r(NULL, ",", &saveptr1);  
			if(read != NULL)
			{
				len = strlen(read);
				if(len < 100)
				{
					strcpy(read1, read);	
					isRead = get_operand(read1, 0);  
				}
			}
			
                }
                else if((strncmp (instr,"inc",3) == 0) ||
			(strncmp (instr,"neg",3) == 0))
                {
                	char *write = strtok_r(NULL, ",", &saveptr1);
			if(write != NULL)
			{
				len = strlen(write);
				if(len < 100)
				{
					strcpy(write1, write);
					isWrite = get_operand(write1, 1);    
				}
				if(isWrite == 0)
				{            	
					strcpy(read1, write1);
					isRead = 0;
				}
			}
                	temp->alu_vul++;
			#ifdef REGION
				if(temp2)
				{
					temp2->alu_vul++;
				}
			#endif
                }
                else
		{
			//MM_FREE((void *)ret.string);
			MM_FREE(ret.string);
			return;
		}

                	
		
                //branch yok 
                
                long v = 0;
                if (isRead == 0)
                	{
                                v = reg_vulnerability(temp->register_vul, temp->instr_num, read1, 1, s);
				++temp->reg_live_area;
				temp->reg_vul = temp->reg_vul + v;
                                strcpy(bt->read_value, read1);
				#ifdef REGION
				if(temp2)
				{
					v = reg_vulnerability(temp2->register_vul, temp2->instr_num, read1, 1, s);
					++temp2->reg_live_area;
					temp2->reg_vul = temp2->reg_vul + v;
				}
				#endif
                        }
                	
                	
                	if(isWrite == 0)
                	{
				v = reg_vulnerability(temp->register_vul, temp->instr_num, write1, 0, s);
				temp->reg_vul = temp->reg_vul + v;
                                strcpy(bt->write_value, write1);
				#ifdef REGION
				if(temp2)
				{
					v = reg_vulnerability(temp2->register_vul, temp2->instr_num, write1, 0, s);
					temp2->reg_vul = temp2->reg_vul + v;
				}
				#endif
			}

                	if(isWrite == 2)//write resource'da [] icinde, read olarak ekliyorum
			{
				v = reg_vulnerability(temp->register_vul, temp->instr_num, write1, 1, s);
				++temp->reg_live_area;
				temp->reg_vul = temp->reg_vul + v;
				#ifdef REGION
				if(temp2)
				{
					v = reg_vulnerability(temp2->register_vul, temp2->instr_num, write1, 1, s);
					++temp2->reg_live_area;
					temp2->reg_vul = temp2->reg_vul + v;
				}
				#endif
                        }
                //MM_FREE((void *)ret.string);
		MM_FREE(ret.string);
        } else {
                //s += vtsprintf(s, "(*** No disassembly provided ***)");
        }
}

int get_operand(char *operand, int write)
{
    if(operand == NULL)
    {
        return 1;
    }
    if(isdigit(operand[0]))//immediate
    {
        return 1;
    }
    int found = 0;
    char* tmp= operand;
    int j, k = 0, i = strlen(operand);
    for (j =0; j<i ;j++)
    {
	
	if (found)
	{
          if(isdigit(operand[j]))
		return 1;
          if ((operand[j]== '*') || (operand[j]== ']'))
		break;
          if (operand[j]== ' ')
	  	continue;	
	  tmp[k++] = tmp[j];  	
	}	
	if(operand[j]== '[')
	   found = 1;	
    }
    if (k)
      operand[k] = 0;	
   
    if (found && (k>4 || k<2))
	return 1;


    if (!found)
    { 
 	    k = 0, i = strlen(operand);

	    for (j =0; j<i ;j++)
	    {
		if(operand[j]!= ' ')
		   operand[k++] = operand[j];	
	    }
	    operand[k] = 0;

	     k = strlen(operand);
	   
	    if ((k>4 || k<2))
		return 1;

     }
   //write ariyorum [ var, bunu read olarak almam gerekiyor
   if(found && write == 1){
	return 2;
   }
   //0x gibileri istemiyorum
   if(operand[0] == '0'){
	return 1;
   }
   
   return 0;

}

static void
text_tracer(base_trace_t *bt, trace_entry_t *ent)
{
	/* With all options and longest opcodes, the lenght is around 200 */
        char s[8192];

        switch (ent->trace_type) {
        case TR_Data:
                text_trace_data(bt, ent, s);
                break;
        case TR_Exception:
                text_trace_exception(bt, ent, s);
                break;
        case TR_Instruction:
		text_trace_instruction(bt, ent, s);
                break;
        case TR_Reserved:
        default:
                strcpy(s, "*** Trace: unknown trace event type.\n");
                break;
        }

	/* Catch errors that produce unreasonable long lines */
	//ASSERT(strlen(s) < 500);
	
        if (bt->file != NULL) {
		fputs(s, bt->file);
        } else if (GZ_FILE(bt) != NULL) {
		GZ(gzwrite(GZ_FILE(bt), s, strlen(s)));
        } else {
		SIM_write(s, strlen(s));
        }
}

static void
raw_tracer(base_trace_t *bt, trace_entry_t *ent)
{
        if (bt->file != NULL) {
                fwrite(ent, sizeof(trace_entry_t), 1, bt->file);
        } else if (GZ_FILE(bt) != NULL) {
                GZ(gzwrite(GZ_FILE(bt), ent, sizeof(trace_entry_t)));
        }
}

#if defined(TRACE_STATS)
static void
stats_tracer(base_trace_t *bt, trace_entry_t *ent)
{
        if (ent->trace_type == TR_Data)
                bt->data_records++;
        else if (ent->trace_type == TR_Instruction)
                bt->instruction_records++;
        else
                bt->other_records++;
}
#endif   /* TRACE_STATS */


/*****************************************************************/

/*
 * Add your own tracers here.
 */

/*****************************************************************/


/* Return true if the memory operation intersects the data range intervals,
   false otherwise. */
static int
data_range_filter(interval_list_t *ivs, generic_transaction_t *mop)
{
        generic_address_t address[NUM_ADDRESS_TYPES];
        interval_t *iv;
        int i;
        int all_ivs_empty = 1;

        address[PHYSICAL] = mop->physical_address;
        address[VIRTUAL] = mop->logical_address;
        for (i = 0; i < NUM_ADDRESS_TYPES; i++) {
                if (VLEN(ivs[i]) == 0)
                        continue;
                all_ivs_empty = 0; /* we have just seen a nonempty interval
                                      list */
                VFOREACH(ivs[i], iv) {
                        if (address[i] + mop->size - 1 >= iv->start
                            && address[i] <= iv->end) {
                                /* Access intersects data range interval: it
                                   passes the filter. */
                                return 1;
                        }
                }
        }

        /* We get here only by not intersecting any data range interval. This
           is a success if and only if there were no intervals. */
        return all_ivs_empty;
}

static cycles_t
trace_mem_hier_operate(conf_object_t *obj, conf_object_t *space,
                       map_list_t *map, generic_transaction_t *mop)
{
        trace_mem_hier_object_t *tmho = (trace_mem_hier_object_t *)obj;
        base_trace_t *bt = tmho->bt;

        /* If the DSTC line(s) that the memop intersects pass the filter, we
           want to see future transactions. */
        if (data_range_filter(bt->data_stc_interval, mop))
                mop->block_STC = 1;

	/* forward operation on to underlying timing_model */
	return (tmho->timing_model == NULL
                ? 0
                : tmho->timing_iface.operate(tmho->timing_model, space, map, mop));
}

static int
is_duplicate(trace_entry_t *a, trace_entry_t *b)
{
        return (a->arch == b->arch
                && a->trace_type == b->trace_type
                && a->cpu_no == b->cpu_no
                && a->read_or_write == b->read_or_write
                && a->va == b->va
                && a->pa == b->pa);
}


static cycles_t
trace_snoop_operate(conf_object_t *obj, conf_object_t *space,
                    map_list_t *map, generic_transaction_t *mop)
{
	trace_mem_hier_object_t *tmho = (trace_mem_hier_object_t *)obj;
        base_trace_t *bt = tmho->bt;
	
	if (!SIM_mem_op_is_data(mop) || mop->type == Sim_Trans_Cache
            || !data_range_filter(bt->data_interval, mop))
                goto forward;

        bt->current_entry.trace_type = TR_Data;
        bt->current_entry.read_or_write =
                SIM_mem_op_is_write(mop) ? Sim_RW_Write : Sim_RW_Read;

        bt->current_entry.value.data = 0;
        bt->current_entry.cpu_no = -1;
	
	/*if(SIM_get_mem_op_user_data(mop) != NULL){
		int *xx = (int *)SIM_get_mem_op_user_data(mop);
		bt->current_entry.L1_hit_or_miss = xx[0];
		bt->current_entry.L2_hit_or_miss = xx[1];
	}
	else
	{
		bt->current_entry.L1_hit_or_miss = 0;
		bt->current_entry.L2_hit_or_miss = 0;
	}*/
	
	if(SIM_get_mem_op_user_data(mop) != NULL){
		bt->current_entry.L1_hit_or_miss = 1;
	}else{
		bt->current_entry.L1_hit_or_miss = 0;
	}
        if (SIM_mem_op_is_from_cpu(mop)) {
                if (mop->size > 0 && mop->size <= sizeof(uint64))
                        bt->current_entry.value.data
                                = SIM_get_mem_op_value_cpu(mop);
                bt->current_entry.cpu_no = SIM_get_processor_number(mop->ini_ptr);
        }

        bt->current_entry.size          = mop->size;
        bt->current_entry.va            = mop->logical_address;
        bt->current_entry.pa            = mop->physical_address;
        bt->current_entry.atomic        = mop->atomic;

        if (mop->ini_type == Sim_Initiator_CPU_X86) {
                x86_memory_transaction_t *xtrans = (void *)mop;
                bt->current_entry.la            = xtrans->linear_address;
                bt->current_entry.linear_access = xtrans->access_linear;
                bt->current_entry.seg_reg       = xtrans->selector;
                bt->current_entry.access_type   = xtrans->access_type;
                bt->current_entry.memory_type   = xtrans->effective_type;
                bt->current_entry.arch          = TA_x86;
        } else if (mop->ini_type == Sim_Initiator_CPU_IA64) {
                bt->current_entry.arch = TA_ia64;
        } else if (SIM_mem_op_is_from_cpu_arch(mop, Sim_Initiator_CPU_V9)) {
                v9_memory_transaction_t *v9_trans = (void *)mop;
                bt->current_entry.arch = TA_v9;
                bt->current_entry.access_type   = v9_trans->access_type;
        } else {
                bt->current_entry.arch = TA_generic;
        }

        cycles_t t = SIM_cycle_count(mop->ini_ptr);
        bt->current_entry.timestamp = t - bt->last_timestamp;
        bt->last_timestamp = t;

        /* If instruction crosses a page call the trace consumer next time. */
        if (mop->page_cross == 1 && SIM_mem_op_is_instruction(mop))
                goto forward;

        /* Give the memory operation to the trace consumer. */
        if (!bt->filter_duplicates
            || !is_duplicate(&bt->last_entry, &bt->current_entry)) {
                bt->last_entry = bt->current_entry;
                bt->trace_consume(bt, &bt->current_entry);
        }

 forward:
        return (tmho->snoop_device == NULL
                ? 0
                : tmho->snoop_iface.operate(tmho->snoop_device, space, map, mop));
}


/* Determine the architecture implemented by the processor object. */
static trace_arch_t
trace_arch_from_cpu(conf_object_t *cpu)
{
        attr_value_t arch = SIM_get_attribute(cpu, "architecture");
        const char *arch_str = SIM_attr_string(arch);
        trace_arch_t ret;

        if (strncmp(arch_str, "x86", 3) == 0) {
                /* x86 or x86-64. */
                ret = TA_x86;
        } else if (strcmp(arch_str, "ia64") == 0) {
                /* ia64. */
                ret = TA_ia64;
        } else if (strcmp(arch_str, "sparc-v9") == 0) {
                /* SPARC V9 */
                ret = TA_v9;
        } else {
                /* Any other architecture. */
                ret = TA_generic;
        }
        
        SIM_attr_free(&arch);
        return ret;
}


static void
trace_instr_operate(lang_void *data, conf_object_t *cpu,
                    linear_address_t la, logical_address_t va,
                    physical_address_t pa, byte_string_t opcode)
{
        base_trace_t *bt = (base_trace_t *) data;

        bt->current_entry.arch = trace_arch_from_cpu(cpu);
        bt->current_entry.trace_type = TR_Instruction;
        bt->current_entry.cpu_no = SIM_get_processor_number(cpu);
        bt->current_entry.size = opcode.len;
        bt->current_entry.read_or_write = Sim_RW_Read;

        if (bt->current_entry.arch == TA_x86) {
                bt->current_entry.linear_access = 0;
                bt->current_entry.seg_reg = 1; /* cs */
                bt->current_entry.la = la;
                bt->current_entry.memory_type = 0;
        }

        bt->current_entry.va = va;
        bt->current_entry.pa = pa;
        bt->current_entry.atomic = 0;
        bt->current_entry.access_type = 0;
        cycles_t t = SIM_cycle_count(cpu);
        bt->current_entry.timestamp = t - bt->last_timestamp;
        bt->last_timestamp = t;

        memcpy(bt->current_entry.value.text, opcode.str, opcode.len);

        bt->last_entry = bt->current_entry;
        bt->trace_consume(bt, &bt->current_entry);
}


/*************** base class *******************/

/* Call when bt->trace_format or bt->file_name have changed. */
static void
raw_mode_onoff_update(base_trace_t *bt)
{
        /* Make sure we are not writing raw output to the console. */
        if (bt->file_name == NULL)
                bt->trace_format = 0; /* text mode */

        /* Change trace consumer to match trace format. */
        if (bt->trace_format == 0)
                bt->trace_consume = text_tracer;
        else
                bt->trace_consume = raw_tracer;
}

/* Decide if this file name indicates that we should use gz compression (by
   looking for ".gz" suffix). */
static int
is_gz_filename(const char *fname)
{
        int len;

        len = strlen(fname);
        return len > 3 && fname[len - 3] == '.' && fname[len - 2] == 'g'
                && fname[len - 1] == 'z';
}

/* Open or close the output file, as appropriate. Call this whenever file_name
   or trace_enabled has changed. */
static set_error_t
file_onoff_update(base_trace_t *bt)
{
        /* Close regular file, if one is open. */
        if (bt->file != NULL)
                fclose(bt->file);
        bt->file = NULL;

#if defined(HAVE_LIBZ)
        /* Close gz file, if one is open. */
        if (GZ_FILE(bt) != NULL)
                gzclose(GZ_FILE(bt));
        GZ_FILE(bt) = NULL;
#endif

        /* Open file if a file name has been set and the tracer is enabled. */
        if (bt->trace_enabled && bt->file_name != NULL) {
                int file_exists = 0;
                if (is_gz_filename(bt->file_name)) {
#if defined(HAVE_LIBZ)
                        /* We have to open the file twice, since gztell always
                           returns 0 for newly opened files. */
                        FILE *f = os_fopen(bt->file_name, "a");
                        file_exists = f && ftell(f) > 0;
                        if (f)
                                os_fclose(f);
                        GZ_FILE(bt) = gzopen(bt->file_name, "a");
#else
                        bt->file_name = NULL;
                        SIM_attribute_error("gzip compression unavailable");
                        return Sim_Set_Illegal_Value;
#endif
                } else {
                        bt->file = os_fopen(bt->file_name,
                                            bt->trace_format == 0 ? "a" : "ab");
                        file_exists = bt->file && ftell(bt->file) > 0;
                }
                if (GZ_FILE(bt) == NULL && bt->file == NULL) {
                        bt->file_name = NULL;
                        SIM_attribute_error("Cannot open file");
                        return Sim_Set_Illegal_Value;
                }
                if (bt->warn_on_existing_file && file_exists) {
                        bt->warn_on_existing_file = 0;
                        SIM_log_info(1, &bt->log, 0,
                                     "Appending trace to existing file %s",
                                     bt->file_name);
                }
        }

        /* Raw output to console is not allowed. */
        raw_mode_onoff_update(bt);

        return Sim_Set_Ok;
}

/* Register or unregister the execution trace callback for all processors,
   depending on the values of trace_enabled and trace_instructions. */
static void
instruction_trace_onoff_update(base_trace_t *bt)
{
        int i;
        attr_value_t all_objects = SIM_get_all_objects();

        for (i = 0; i < all_objects.u.list.size; i++) {
                conf_object_t *obj = all_objects.u.list.vector[i].u.object;
                exec_trace_interface_t *iface = 
                        SIM_c_get_interface(obj, EXEC_TRACE_INTERFACE);
                if (iface) {
                        if (bt->trace_enabled && bt->trace_instructions)
                                iface->register_tracer(
                                        obj, trace_instr_operate, bt);
                        else
                                iface->unregister_tracer(
                                        obj, trace_instr_operate, bt);
                } 
        }
	print_results(bt);

        SIM_attr_free(&all_objects);
	
}

/*void reg_log(int count)
{
	FILE *s;
	s = fopen("reg_log.txt", "w");
	fprintf(s, "%d",count);
	fclose(s);
}

void mem_log(int count)
{
	FILE *s;
	s = fopen("mem_log.txt", "w");
	fprintf(s, "%d",count);
	fclose(s);
}

void remote_log(int count)
{
	FILE *s;
	s = fopen("remote_log.txt", "w");
	fprintf(s, "%d",count);
	fclose(s);
}

void store_log(int count)
{
	FILE *s;
	s = fopen("store_log.txt", "w");
	fprintf(s, "%d",count);
	fclose(s);
}

void write_log(char *str)
{
	FILE *s;
	s = fopen("trace_error.txt", "w");
	fprintf(s, "%s\n",str);
	fclose(s);
}*/

void print_fan_info(Fan_info *fan, FILE *s)
{
        int critical = 0;
        int critical_direct = 0;
        int critical_indirect = 0;
        int i,j;
        double max = 0;
        double max_direct = 0;
        double max_indirect = 0;
        for(i = 0; i < MAX_THREADS; i++)
        {
                double local = 0;
                double locald = 0;
                double localid = 0;
                for(j = 0; j < MAX_THREADS; j++)
                {
                        locald += fan->direct_reg_array[i][j];
                        localid += fan->indirect_reg_array[i][j];
                        fprintf(s, "DIRECT_REG_ARRAY[%d][%d]=%f\n",i,j,fan->direct_reg_array[i][j]);
                        fprintf(s, "INDIRECT_REG_ARRAY[%d][%d]=%f\n",i,j,fan->indirect_reg_array[i][j]);
                        local = locald + localid;
                }
                fprintf(s, "%d direct local register sum: %f\n",i,locald);
                fprintf(s, "%d indirect local register sum: %f\n",i,localid);
                fprintf(s, "%d total local register sum: %f\n",i,local);

                if(local > max)
                {
                        max = local;
                        critical = i;
                }

                if(locald > max_direct)
                {
                        max_direct = locald;
                        critical_direct = i;
                }

                if(localid > max_indirect)
                {
                        max_indirect = localid;
                        critical_indirect = i;
                }
        }
        fprintf(s, "The most critical for register: %d\n",critical);
        fprintf(s, "The most critical for direct register: %d\n",critical_direct);
        fprintf(s, "The most critical for indirect register: %d\n",critical_indirect);

        critical = 0; max = 0;
        critical_direct = 0; max_direct = 0;
        critical_indirect = 0; max_indirect = 0;

        for(i = 0; i < MAX_THREADS; i++)
        {
                double local = 0;
                double locald = 0;
                double localid = 0;
                for(j = 0; j < MAX_THREADS; j++)
                {
                        locald += fan->direct_mem_array[i][j];
                        localid += fan->indirect_mem_array[i][j];
                        fprintf(s, "DIRECT_MEM_ARRAY[%d][%d]=%f\n",i,j,fan->direct_mem_array[i][j]);
                        fprintf(s, "INDIRECT_MEM_ARRAY[%d][%d]=%f\n",i,j,fan->indirect_mem_array[i][j]);
                        local = locald + localid;
                }
                fprintf(s, "%d direct local memory sum: %f\n",i,locald);
                fprintf(s, "%d indirect local memory sum: %f\n",i,localid);
                fprintf(s, "%d total local memory sum: %f\n",i,local);

                if(local > max)
                {
                        max = local;
                        critical = i;
                }

                if(locald > max_direct)
                {
                        max_direct = locald;
                        critical_direct = i;
                }

                if(localid > max_indirect)
                {
                        max_indirect = localid;
                        critical_indirect = i;
                }
        }
        fprintf(s, "The most critical for memory: %d\n",critical);
        fprintf(s, "The most critical for direct memory: %d\n",critical_direct);
        fprintf(s, "The most critical for indirect memory: %d\n",critical_indirect);

        critical = 0; max = 0;
        critical_direct = 0; max_direct = 0;
        critical_indirect = 0; max_indirect = 0;

        for(i = 0; i < MAX_THREADS; i++)
        {
                double local = 0;
                double locald = 0;
                double localid = 0;
                for(j = 0; j < MAX_THREADS; j++)
                {
                        locald += fan->direct_alu_array[i][j];
                        localid += fan->indirect_alu_array[i][j];
                        fprintf(s, "DIRECT_ALU_ARRAY[%d][%d]=%f\n",i,j,fan->direct_alu_array[i][j]);
                        fprintf(s, "INDIRECT_ALU_ARRAY[%d][%d]=%f\n",i,j,fan->indirect_alu_array[i][j]);
                        local = locald + localid;
                }
                fprintf(s, "%d direct local alu sum: %f\n",i,locald);
                fprintf(s, "%d indirect local alu sum: %f\n",i,localid);
                fprintf(s, "%d total local alu sum: %f\n",i,local);

                if(local > max)
                {
                        max = local;
                        critical = i;
                }

                if(locald > max_direct)
                {
                        max_direct = locald;
                        critical_direct = i;
                }

                if(localid > max_indirect)
                {
                        max_indirect = localid;
                        critical_indirect = i;
                }
        }
        fprintf(s, "The most critical for alu: %d\n",critical);
        fprintf(s, "The most critical for direct alu: %d\n",critical_direct);
        fprintf(s, "The most critical for indirect alu: %d\n",critical_indirect);

        critical = 0; max = 0;
        critical_direct = 0; max_direct = 0;
        critical_indirect = 0; max_indirect = 0;

        for(i = 0; i < MAX_THREADS; i++)
        {
                double local = 0;
                double locald = 0;
                double localid = 0;
                for(j = 0; j < MAX_THREADS; j++)
                {
                        locald += fan->direct_count[i][j];
                        localid += fan->indirect_count[i][j];
                        fprintf(s, "DIRECT_COUNT_ARRAY[%d][%d]=%f\n",i,j,fan->direct_count[i][j]);
                        fprintf(s, "INDIRECT_COUNT_ARRAY[%d][%d]=%f\n",i,j,fan->indirect_count[i][j]);
                        local = locald + localid;
                }
                fprintf(s, "%d direct local count sum: %f\n",i,locald);
                fprintf(s, "%d indirect local count sum: %f\n",i,localid);
                fprintf(s, "%d total local count sum: %f\n",i,local);

                if(local > max)
                {
                        max = local;
                        critical = i;
                }

                if(locald > max_direct)
                {
                        max_direct = locald;
                        critical_direct = i;
                }

                if(localid > max_indirect)
                {
                        max_indirect = localid;
                        critical_indirect = i;
                }
        }
        fprintf(s, "The most critical for count: %d\n",critical);
        fprintf(s, "The most critical for direct count: %d\n",critical_direct);
        fprintf(s, "The most critical for indirect count: %d\n",critical_indirect);
}

void
print_results(base_trace_t *bt)
{
	FILE *s;
	s = fopen("vul_result.txt", "w");

	for(int i = 0; i < MAX_THREADS; i++)
	{
		printf("print_results:%d\n", i);	
		print_cpu(&bt->threads[i], s);
	}
	print_cpu(&bt->rr_thread, s);
        print_fan_info(bt->fan, s);
	fclose(s);

}

void
print_temp_results(base_trace_t *bt)
{
	FILE *s;
	s = fopen("temp_result.txt", "a");
	fprintf(s, "Region %d\n",bt->region);
	for(int i = 0; i < MAX_THREADS; i++)
	{
		printf("print_results:%d\n", i);	
		print_cpu(&bt->threads[i], s);
	}

        print_fan_info(bt->fan, s);
	fclose(s);

}

void print_cpu(thread_t *cpu, FILE *s)
{
	if(cpu != NULL){
	//printf("print_cpu not null:%d\n", cpu->thread_num);	
	StrMap *reg = cpu->register_vul;
	StrMap *mem = cpu->memory_vul;
	StrMap *remote_reg_vul = cpu->remote_reg_vul;
	StrMap *remote_mem_vul = cpu->remote_mem_vul;
	StrMap *remote_alu_vul = cpu->remote_alu_vul;
	StrMap *remote_count_vul = cpu->remote_count_vul;

	double memm = 0;
	double alu = 0;
	double regg = 0;

        double memm_self = 0;
        double alu_self = 0;
        double regg_self = 0;

	long long reg_area = 1, mem_area = 1;
	if(cpu->reg_live_area != 0) reg_area = cpu->reg_live_area;
	if(cpu->mem_live_area != 0) mem_area = cpu->mem_live_area;
        long long rcount = strmap_get_count(reg);
        long long mcount = strmap_get_count(mem);
	
	double xr = (double)(cpu->instr_num) * (double)rcount;
	double xm = (double)(cpu->instr_num) * (double)mcount;
	//printf("Inst num:%ld\n", cpu->instr_num);
	//printf("MEM: %f live: %ld mul:%f\n", cpu->mem_vul, mem_area, xm);
	//printf("REG: %f live: %ld mul:%f\n", cpu->reg_vul,reg_area, xr);
	
        //0 değerine sahipse 0 olmayan(?) son değeri alalım
        cpu->alu_self_vul = cpu->alu_vul - cpu->alu_self_vul;
        if(cpu->alu_self_vul == 0)
                cpu->alu_self_vul = cpu->alu_vul - cpu->alu_last_remote;

        cpu->reg_self_vul = cpu->reg_vul - cpu->reg_self_vul;
        if(cpu->reg_self_vul == 0)
                cpu->reg_self_vul = cpu->reg_vul - cpu->reg_last_remote;

        cpu->mem_self_vul = cpu->mem_vul - cpu->mem_self_vul;
        if(cpu->mem_self_vul == 0)
                cpu->mem_self_vul = cpu->mem_vul - cpu->mem_last_remote;

	if((cpu->instr_num) != 0){
		alu = (cpu->alu_vul) / (cpu->instr_num);
		regg = (cpu->reg_vul) / xr;
		memm = (cpu->mem_vul) / xm;
                alu_self = (cpu->alu_self_vul) / (cpu->instr_num);
                regg_self = (cpu->reg_self_vul) / xr;
                memm_self = (cpu->mem_self_vul) / xm;

	}
	//printf("register:%f memory:%f\n", regg, memm);
	fprintf(s, "Thread %d\n",cpu->thread_index);
	fprintf(s, "PID %d\n",cpu->thread_num);
	fprintf(s, "Core %d\n",cpu->core_map);
	fprintf(s, "Total instr: %lld\n", cpu->instr_num);
	fprintf(s, "Total count: %lld\n", cpu->instr_count);
	fprintf(s, "Register live: %lld\n", reg_area);
	fprintf(s, "Memory live: %lld\n", mem_area);
	fprintf(s, "Total Remote count: %lld\n", cpu->remote_count);

	fprintf(s, "Self ALU: %f\n",cpu->alu_self_vul);
        fprintf(s, "Self Register: %f\n",cpu->reg_self_vul);
        fprintf(s, "Self Memory: %f\n",cpu->mem_self_vul);
        
	fprintf(s, "ALU: %f\n",alu_self);
        fprintf(s, "Register: %f\n",regg_self);
        fprintf(s, "Memory: %f\n",memm_self);

        fprintf(s, "Complete ALU: %f\n",alu);
        fprintf(s, "Complete Register: %f\n",regg);
        fprintf(s, "Complete Memory: %f\n",memm);

	double rmemm = 0, ralu = 0, rregg = 0;
	if(cpu->remote_count !=0){
		ralu = cpu->rem_alu_vul / (double)cpu->remote_count;
		rmemm = cpu->rem_mem_vul / (double)cpu->remote_count;
		rregg = cpu->rem_reg_vul / (double)cpu->remote_count;
	}
	
	fprintf(s, "Remote ALU: %f\n", ralu); 
	print_strmap(remote_alu_vul, s);
	fprintf(s, "Remote Register: %f\n", rregg);
	print_strmap(remote_reg_vul, s);
	fprintf(s, "Remote Memory: %f\n", rmemm);
	print_strmap(remote_mem_vul, s);
	fprintf(s, "Remote Count: \n");
	print_strmap(remote_count_vul, s);
	fprintf(s, "Register all :\n");
	print_strmap(reg, s);
	fprintf(s, "Memory all:\n");
	//print_strmap(mem, s);

	
	fprintf(s, "************\n\n");
	}else{
		printf("print_cpu null\n");	
	}
}

/* Called for each exception, just prior to servicing them. */
static void
catch_exception_hook(lang_void *data, conf_object_t *cpu, integer_t exc)
{
        base_trace_t *bt = (base_trace_t *) data;

        bt->current_entry.trace_type = TR_Exception;
        bt->current_entry.value.exception = exc;
        bt->current_entry.cpu_no = SIM_get_processor_number(cpu);
        cycles_t t = SIM_cycle_count(cpu);
        bt->current_entry.timestamp = t - bt->last_timestamp;
        bt->last_timestamp = t;

        /* Call the trace handler. */
        bt->last_entry = bt->current_entry;
        bt->trace_consume(bt, &bt->current_entry);
}

/* Called at Simics exit. */
static void
at_exit_hook(lang_void *data)
{
        base_trace_t *bt = (base_trace_t *) data;

        /* Close any open files. */
        bt->file_name = NULL;
        file_onoff_update(bt);
}

/* Register or unregister the exception callback, depending on the values of
   trace_enabled and trace_exceptions. */
static void
exception_trace_onoff_update(base_trace_t *bt)
{
        obj_hap_func_t f = (obj_hap_func_t) catch_exception_hook;

        if (bt->trace_enabled && bt->trace_exceptions)
                SIM_hap_add_callback("Core_Exception", f, bt);
        else
                SIM_hap_delete_callback("Core_Exception", f, bt);
}

/* Data structure for get_all_mem_spaces function */
struct memspace_list {
	conf_object_t *space;
	conf_object_t *cpu;
	struct memspace_list *next;
};

static void
memspace_list_add(struct memspace_list **headp, conf_object_t *space,
		  conf_object_t *cpu)
{
	struct memspace_list *p;

	/* check if already in list */
	for (p = *headp; p; p = p->next)
		if (p->space == space) return;

	p = MM_ZALLOC(1, struct memspace_list);
	p->next = *headp;
	p->space = space;
	p->cpu = cpu;
	*headp = p;
}

/* Find all memory-spaces connected to any cpu. Returns a linked list. */
static struct memspace_list *
find_memspaces(void)
{
        conf_object_t *cpu;
	struct memspace_list *spaces = NULL;

        attr_value_t ifaces = 
                SIM_make_attr_list(
                        1, 
                        SIM_make_attr_string(PROCESSOR_INFO_INTERFACE));
        attr_value_t queues = VT_get_all_objects_impl(ifaces);
        SIM_attr_free(&ifaces);
	
        for (int i = 0; i < SIM_attr_list_size(queues); i++) {
                attr_value_t phys_attr, phys_io;
                cpu = SIM_attr_object(SIM_attr_list_item(queues, i));

                phys_attr = SIM_get_attribute(cpu, "physical_memory");
                SIM_clear_exception();

                phys_io = SIM_get_attribute(cpu, "physical_io");
                SIM_clear_exception();

                /* Clocks does not have a physical_memory object. */
                if (phys_attr.kind == Sim_Val_Object)
                        memspace_list_add(&spaces, phys_attr.u.object, cpu);

                /* SPARC has physical-io. */
                if (phys_io.kind == Sim_Val_Object)
			memspace_list_add(&spaces, phys_io.u.object, cpu);
        }
        SIM_attr_free(&queues);

	return spaces;
}

/* Logical xor. True if exactly one of a and b are true, otherwise false. */
#define LXOR(a, b) ((!!(a) ^ !!(b)) != 0)

/* Register or unregister memory transaction snoopers, depending on the values
   of trace_enabled and trace_data. Raise a frontend exception and return a
   suitable error code in case of failure. */
static set_error_t
data_trace_onoff_update(base_trace_t *bt)
{
        struct memspace_list *spaces, *iter;
        attr_value_t attr;
        trace_mem_hier_object_t *tmho;

        /* Assume no error until proven otherwise. */
        const char *err = NULL;
        set_error_t ret = Sim_Set_Ok;

        if (!LXOR(bt->memhier_hook, bt->trace_enabled && bt->trace_data)) {
                /* This means that we are already hooked into the memory
                   hierarchy if and only if we need to be. So, nothing more to
                   do. */
                return ret;
        }

        /* Find all memory-spaces connected to any cpu. */
	spaces = find_memspaces();
	
        if (!bt->memhier_hook) {
                /* We are not hooked into the memory hierarchy, but we need to
                   be. */

                char buf[128];
                conf_class_t *trace_class;
                int i = 0;

                /* Invalidate any cached memory translations, so that we see
                   all memory operations. */
                SIM_flush_all_caches();

                trace_class = SIM_get_class("trace-mem-hier");

                /* Create mem hiers for every memory-space we will trace on. */
                for (iter = spaces; iter; iter = iter->next) {
                        conf_object_t *space = iter->space;
                        attr_value_t prev_tm, prev_sd;

                        /* Reuse existing object if there is one. */
                        vtsprintf(buf, "trace-mem-hier-%d", i++);
                        tmho = (trace_mem_hier_object_t *) SIM_get_object(buf);
                        if (tmho == NULL) {
                                SIM_clear_exception();
                                /* No, we'll have to create it here. */
                                tmho = (trace_mem_hier_object_t *)
                                        SIM_create_object(trace_class, buf,
                                                        SIM_make_attr_list(0));
                                if (tmho == NULL) {
                                        err = "Cannot create trace object";
                                        ret = Sim_Set_Illegal_Value;
                                        goto finish;
                                }
                        }
                        tmho->bt = bt;
                        tmho->obj.queue = iter->cpu;

			/* Set up forwarding. */
                        tmho->timing_model = NULL;
                        tmho->snoop_device = NULL;
                        memset(&tmho->timing_iface, 0,
                               sizeof(tmho->timing_iface));
                        memset(&tmho->snoop_iface, 0,
                               sizeof(tmho->snoop_iface));
                        prev_tm = SIM_get_attribute(space, "timing_model");
                        if (prev_tm.kind == Sim_Val_Object) {
                                timing_model_interface_t *timing_iface;
                                timing_iface = SIM_c_get_interface(
                                        prev_tm.u.object,
                                        TIMING_MODEL_INTERFACE);
                                if (timing_iface != NULL) {
                                        tmho->timing_model = prev_tm.u.object;
                                        tmho->timing_iface = *timing_iface;
                                }
                        }
                        prev_sd = SIM_get_attribute(space, "snoop_device");
                        if (prev_sd.kind == Sim_Val_Object) {
                                timing_model_interface_t *snoop_iface;
                                snoop_iface = SIM_c_get_interface(
                                        prev_sd.u.object,
                                        SNOOP_MEMORY_INTERFACE);
                                if (snoop_iface != NULL) {
                                        tmho->snoop_device = prev_sd.u.object;
                                        tmho->snoop_iface = *snoop_iface;
                                }
                        }

                        /* May have exceptions after
                           SIM_get_attribute. We don't care. */
                        SIM_clear_exception();

                        /* Plug in new memory hierarchy. */
                        attr = SIM_make_attr_object(&tmho->obj);
                        SIM_set_attribute(space, "snoop_device", &attr);
                        if (SIM_clear_exception() != SimExc_No_Exception) {
                                err = "Could not install snoop device";
                                ret = Sim_Set_Illegal_Value;
                                goto finish;
                        }
                        SIM_set_attribute(space, "timing_model", &attr);
                        if (SIM_clear_exception() != SimExc_No_Exception) {
                                err = "Could not install timing model";
                                ret = Sim_Set_Illegal_Value;
                                goto finish;
                        }
                }
        } else {
                /* We are hooked into the memory hierarchy, but we don't need
                   to be. So splice out the trace_mem_hier_object_t from
                   phys_mem.timing_model and phys_mem.snoop_device. */
                for (iter = spaces; iter; iter = iter->next) {
                        conf_object_t *space = iter->space;

                        attr = SIM_get_attribute(space, "timing_model");
                        tmho = (trace_mem_hier_object_t *) 
                                SIM_attr_object(attr);
                        attr = SIM_make_attr_object(tmho->snoop_device);
                        SIM_set_attribute(space, "snoop_device", &attr);
                        if (SIM_clear_exception() != SimExc_No_Exception) {
                                err = "Could not uninstall snoop device";
                                ret = Sim_Set_Illegal_Value;
                                goto finish;
                        }
                        attr = SIM_make_attr_object(tmho->timing_model);
                        SIM_set_attribute(space, "timing_model", &attr);
                        if (SIM_clear_exception() != SimExc_No_Exception) {
                                err = "Could not uninstall timing model";
                                ret = Sim_Set_Illegal_Value;
                                goto finish;
                        }
                }
        }

        /* If we get here, we changed state. */
        bt->memhier_hook = !bt->memhier_hook;

  finish:
        if (err != NULL)
                SIM_attribute_error(err);

	/* free the linked list */
	for (iter = spaces; iter; ) {
		struct memspace_list *next = iter->next;
		MM_FREE(iter);
		iter = next;
	}
        return ret;
}

static set_error_t
set_redundant_region(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;
        bt->redundant_region = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_redundant_region(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->redundant_region);
}

static set_error_t
set_redundant_thread(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;
        bt->redundant_thread = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_redundant_thread(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->redundant_thread);
}

static set_error_t
set_partial_result(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;
        bt->partial_result = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_partial_result(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->partial_result);
}

static set_error_t
set_fault_injection_instruction(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;
	bt->fault_injection_instruction = atoll(val->u.string);
        printf("Fault injection instruction %lld\n", bt->fault_injection_instruction);
        return Sim_Set_Ok;
}

static attr_value_t
get_fault_injection_instruction(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_string(bt->fault_injection_instruction);
}

static set_error_t
set_fault_injection_core(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;
        bt->fault_injection_core = val->u.integer;
        printf("Fault injection core %d\n", bt->fault_injection_core);
        return Sim_Set_Ok;
}

static attr_value_t
get_fault_injection_core(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->fault_injection_core);
}

static set_error_t
set_redundant(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;
        bt->redundant = val->u.integer;
        printf("Redundant thread no %d\n", bt->redundant);
        return Sim_Set_Ok;
}

static attr_value_t
get_redundant(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->redundant);
}

static set_error_t
set_cpu0_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu0_active != !!val->u.integer) {
                bt->cpu0_active = !!val->u.integer;
        }
	bt->cores[0].active = val->u.integer;
	return Sim_Set_Ok;
}

static attr_value_t
get_cpu0_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
	return SIM_make_attr_integer(bt->cpu0_active);
	//return *idx;
}

static set_error_t
set_cpu1_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu1_active != !!val->u.integer) {
                bt->cpu1_active = !!val->u.integer;
        }
	bt->cores[1].active = val->u.integer;
	return Sim_Set_Ok;
}

static attr_value_t
get_cpu1_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu1_active);
	//return *idx;
}

static set_error_t
set_cpu2_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu2_active != !!val->u.integer) {
                bt->cpu2_active = !!val->u.integer;
        }
	bt->cores[2].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu2_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu2_active);
	//return *idx;
}

static set_error_t
set_cpu3_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu3_active != !!val->u.integer) {
                bt->cpu3_active = !!val->u.integer;
        }
	bt->cores[3].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu3_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu3_active);
	//return *idx;
}

static set_error_t
set_cpu4_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu4_active != !!val->u.integer) {
                bt->cpu4_active = !!val->u.integer;
        }
	bt->cores[4].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu4_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu4_active);
	//return *idx;
}

static set_error_t
set_cpu5_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu5_active != !!val->u.integer) {
                bt->cpu5_active = !!val->u.integer;
        }
	bt->cores[5].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu5_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu5_active);
	//return *idx;
}

static set_error_t
set_cpu6_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu6_active != !!val->u.integer) {
                bt->cpu6_active = !!val->u.integer;
        }
	bt->cores[6].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu6_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu6_active);
	//return *idx;
}

static set_error_t
set_cpu7_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu7_active != !!val->u.integer) {
                bt->cpu7_active = !!val->u.integer;
        }
	bt->cores[7].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu7_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu7_active);
	//return *idx;
}

static set_error_t
set_cpu8_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu8_active != !!val->u.integer) {
                bt->cpu8_active = !!val->u.integer;
        }
	bt->cores[8].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu8_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu8_active);
	//return *idx;
}

static set_error_t
set_cpu9_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu9_active != !!val->u.integer) {
                bt->cpu9_active = !!val->u.integer;
        }
	bt->cores[9].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu9_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu9_active);
	//return *idx;
}

static set_error_t
set_cpu10_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu10_active != !!val->u.integer) {
                bt->cpu10_active = !!val->u.integer;
        }
	bt->cores[10].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu10_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu10_active);
	//return *idx;
}

static set_error_t
set_cpu11_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu11_active != !!val->u.integer) {
                bt->cpu11_active = !!val->u.integer;
        }
	bt->cores[11].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu11_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu11_active);
	//return *idx;
}

static set_error_t
set_cpu12_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu12_active != !!val->u.integer) {
                bt->cpu12_active = !!val->u.integer;
        }
	bt->cores[12].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu12_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu12_active);
	//return *idx;
}

static set_error_t
set_cpu13_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu13_active != !!val->u.integer) {
                bt->cpu13_active = !!val->u.integer;
        }
	bt->cores[13].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu13_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu13_active);
	//return *idx;
}

static set_error_t
set_cpu14_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu14_active != !!val->u.integer) {
                bt->cpu14_active = !!val->u.integer;
        }
	bt->cores[14].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu14_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu14_active);
	//return *idx;
}

static set_error_t
set_cpu15_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu15_active != !!val->u.integer) {
                bt->cpu15_active = !!val->u.integer;
        }
	bt->cores[15].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu15_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu15_active);
	//return *idx;
}

static set_error_t
set_cpu16_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu16_active != !!val->u.integer) {
                bt->cpu16_active = !!val->u.integer;
        }
	bt->cores[16].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu16_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu16_active);
	//return *idx;
}

static set_error_t
set_cpu17_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu17_active != !!val->u.integer) {
                bt->cpu17_active = !!val->u.integer;
        }
	bt->cores[17].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu17_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu17_active);
	//return *idx;
}

static set_error_t
set_cpu18_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu18_active != !!val->u.integer) {
                bt->cpu18_active = !!val->u.integer;
        }
	bt->cores[18].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu18_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu18_active);
	//return *idx;
}

static set_error_t
set_cpu19_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu19_active != !!val->u.integer) {
                bt->cpu19_active = !!val->u.integer;
        }
	bt->cores[19].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu19_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu19_active);
	//return *idx;
}

static set_error_t
set_cpu20_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu20_active != !!val->u.integer) {
                bt->cpu20_active = !!val->u.integer;
        }
	bt->cores[20].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu20_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu20_active);
	//return *idx;
}

static set_error_t
set_cpu21_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu21_active != !!val->u.integer) {
                bt->cpu21_active = !!val->u.integer;
        }
	bt->cores[21].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu21_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu21_active);
	//return *idx;
}

static set_error_t
set_cpu22_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu22_active != !!val->u.integer) {
                bt->cpu22_active = !!val->u.integer;
        }
	bt->cores[22].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu22_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu22_active);
	//return *idx;
}

static set_error_t
set_cpu23_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu23_active != !!val->u.integer) {
                bt->cpu23_active = !!val->u.integer;
        }
	bt->cores[23].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu23_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu23_active);
	//return *idx;
}

static set_error_t
set_cpu24_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu24_active != !!val->u.integer) {
                bt->cpu24_active = !!val->u.integer;
        }
	bt->cores[24].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu24_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu24_active);
	//return *idx;
}

static set_error_t
set_cpu25_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu25_active != !!val->u.integer) {
                bt->cpu25_active = !!val->u.integer;
        }
	bt->cores[25].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu25_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu25_active);
	//return *idx;
}

static set_error_t
set_cpu26_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu26_active != !!val->u.integer) {
                bt->cpu26_active = !!val->u.integer;
        }
	bt->cores[26].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu26_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu26_active);
	//return *idx;
}

static set_error_t
set_cpu27_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu27_active != !!val->u.integer) {
                bt->cpu27_active = !!val->u.integer;
        }
	bt->cores[27].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu27_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu27_active);
	//return *idx;
}

static set_error_t
set_cpu28_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu28_active != !!val->u.integer) {
                bt->cpu28_active = !!val->u.integer;
        }
	bt->cores[28].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu28_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu28_active);
	//return *idx;
}

static set_error_t
set_cpu29_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu29_active != !!val->u.integer) {
                bt->cpu29_active = !!val->u.integer;
        }
	bt->cores[29].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu29_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu29_active);
	//return *idx;
}

static set_error_t
set_cpu30_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu30_active != !!val->u.integer) {
                bt->cpu30_active = !!val->u.integer;
        }
	bt->cores[30].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu30_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu30_active);
	//return *idx;
}

static set_error_t
set_cpu31_active(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->cpu31_active != !!val->u.integer) {
                bt->cpu31_active = !!val->u.integer;
        }
	bt->cores[31].active = val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_cpu31_active(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->cpu31_active);
	//return *idx;
}

static set_error_t
set_raw(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;

        bt->trace_format = !!val->u.integer;
        raw_mode_onoff_update(bt);
        if (bt->trace_format != !!val->u.integer) {
                /* Change to raw mode was vetoed. */
                SIM_attribute_error("Raw output must be written to a file");
                return Sim_Set_Illegal_Value;
        }
        return Sim_Set_Ok;
}


static attr_value_t
get_raw(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->trace_format);
}


static set_error_t
set_consumer(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        trace_consume_interface_t *consume_iface;

        if (val->kind == Sim_Val_Nil) {
                bt->consumer = NULL;
                if (bt->trace_format == 0)
                        bt->trace_consume = text_tracer;
                else
                        bt->trace_consume = raw_tracer;
                return Sim_Set_Ok;
        }

        consume_iface = SIM_c_get_interface(val->u.object, 
                                            TRACE_CONSUME_INTERFACE);
        if (!consume_iface)
                return Sim_Set_Interface_Not_Found;

        bt->consume_iface = *consume_iface;
        bt->consumer = val->u.object;
        bt->trace_consume = external_tracer;
        return Sim_Set_Ok;
}


static attr_value_t
get_consumer(void *arg, conf_object_t *obj, attr_value_t *idx)
{
	base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_object(bt->consumer);
}


#if defined(TRACE_STATS)
static set_error_t
set_instruction_records(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->instruction_records = val->u.integer;
        return Sim_Set_Ok;
}


static attr_value_t
get_instruction_records(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->instruction_records);
}


static set_error_t
set_data_records(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->data_records = val->u.integer;
        return Sim_Set_Ok;
}


static attr_value_t
get_data_records(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->data_records);
}


static set_error_t
set_other_records(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->other_records = val->u.integer;
        return Sim_Set_Ok;
}


static attr_value_t
get_other_records(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->other_records);
}
#endif   /* TRACE_STATS */

static set_error_t
set_file(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;
        set_error_t ret;
        char *old_fn = bt->file_name;

        if (val->kind == Sim_Val_String)
                bt->file_name = MM_STRDUP(val->u.string);
        else
                bt->file_name = NULL;

        /* Try to open/close file. */
        if ((!old_fn && bt->file_name)
            || (old_fn && bt->file_name && strcmp(old_fn, bt->file_name) != 0))
                bt->warn_on_existing_file = 1;
        ret = file_onoff_update(bt);
        if (ret != Sim_Set_Ok) {
                /* Failed, set to no file. */
                if (bt->file_name != NULL)
                        MM_FREE(bt->file_name);
                bt->file_name = NULL;
        }

        MM_FREE(old_fn);
        return ret;
}

static attr_value_t
get_file(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
	return SIM_make_attr_string(bt->file_name);
}


static set_error_t
set_enabled(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        set_error_t ret;

        if (!!bt->trace_enabled == !!val->u.integer)
                return Sim_Set_Ok; /* value did not change */

        /* Change the enabled state and try to start or stop the data
           tracing. */
        bt->trace_enabled = !!val->u.integer;
        ret = data_trace_onoff_update(bt);
        if (ret == Sim_Set_Ok) {
                /* Success, start or stop the other tracers as well. */
                instruction_trace_onoff_update(bt);
                exception_trace_onoff_update(bt);
                file_onoff_update(bt);
        } else {
                /* Failure, revert the change. */
                bt->trace_enabled = !bt->trace_enabled;
        }

        return ret;
}


static attr_value_t
get_enabled(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->trace_enabled);
}

static set_error_t
set_trace_instructions(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->trace_instructions != !!val->u.integer) {
                bt->trace_instructions = !!val->u.integer;
                instruction_trace_onoff_update(bt);
        }
        return Sim_Set_Ok;
}


static attr_value_t
get_trace_instructions(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->trace_instructions);
}


static set_error_t
set_trace_data(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->trace_data != !!val->u.integer) {
                bt->trace_data = !!val->u.integer;
                return data_trace_onoff_update(bt);
        } else {
                return Sim_Set_Ok;
        }
}


static attr_value_t
get_trace_data(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->trace_data);
}


static set_error_t
set_trace_exceptions(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *) obj;

        /* Update if state changed. */
        if (!!bt->trace_exceptions != !!val->u.integer) {
                bt->trace_exceptions = !!val->u.integer;
                exception_trace_onoff_update(bt);
        }
        return Sim_Set_Ok;
}


static attr_value_t
get_trace_exceptions(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->trace_exceptions);
}


static set_error_t
set_filter_duplicates(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->filter_duplicates = !!val->u.integer;
        return Sim_Set_Ok;
}


static attr_value_t
get_filter_duplicates(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->filter_duplicates);
}


static set_error_t
set_print_virtual_address(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->print_virtual_address = !!val->u.integer;
        return Sim_Set_Ok;
}


static attr_value_t
get_print_virtual_address(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->print_virtual_address);
}


static set_error_t
set_print_physical_address(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->print_physical_address = !!val->u.integer;
        return Sim_Set_Ok;
}


static attr_value_t
get_print_physical_address(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->print_physical_address);
}


static set_error_t
set_print_linear_address(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->print_linear_address = !!val->u.integer;
        return Sim_Set_Ok;
}


static attr_value_t
get_print_linear_address(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->print_linear_address);
}


static set_error_t
set_print_access_type(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->print_access_type = !!val->u.integer;
        return Sim_Set_Ok;
}

static attr_value_t
get_print_access_type(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->print_access_type);
}


static set_error_t
set_print_memory_type(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->print_memory_type = !!val->u.integer;
        return Sim_Set_Ok;
}


static attr_value_t
get_print_memory_type(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->print_memory_type);
}


static set_error_t
set_print_data(void *arg, conf_object_t *obj, attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        bt->print_data = !!val->u.integer;
        return Sim_Set_Ok;
}


static attr_value_t
get_print_data(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        return SIM_make_attr_integer(bt->print_data);
}


static attr_value_t
get_base_trace(void *arg, conf_object_t *obj, attr_value_t *idx)
{
        trace_mem_hier_object_t *tmho = (trace_mem_hier_object_t *)obj;
        return SIM_make_attr_object(&tmho->bt->log.obj);
}


static attr_value_t
get_timing_model(void *arg, conf_object_t *obj, attr_value_t *idx)
{
	printf("get_timing_model\n");
        trace_mem_hier_object_t *tmho = (trace_mem_hier_object_t *)obj;
        return SIM_make_attr_object(tmho->timing_model);
}


static attr_value_t
get_snoop_device(void *arg, conf_object_t *obj, attr_value_t *idx)
{
	printf("get_snoop_device\n");
        trace_mem_hier_object_t *tmho = (trace_mem_hier_object_t *)obj;
        return SIM_make_attr_object(tmho->snoop_device);
}


/* Create a new interval_t. Round outwards to the specified number of bits. */
static interval_t
create_interval(generic_address_t start, generic_address_t end, int round)
{
        interval_t iv;
        generic_address_t rmask = ~(generic_address_t)((1 << round) - 1);

        iv.start = MIN(start, end) & rmask;
        iv.end = (MAX(start, end) & rmask) + (1 << round) - 1;
        return iv;
}

static set_error_t
set_data_intervals(void *arg, conf_object_t *obj,
                   attr_value_t *val, attr_value_t *idx)
{
        base_trace_t *bt = (base_trace_t *)obj;
        int address_type = (uintptr_t)arg;
        int i, stc_block;

        /* Find largest DSTC block size. */
        stc_block = 0;
        for (i = 0; i < SIM_number_processors(); i++)
                stc_block = MAX(stc_block, SIM_attr_integer(SIM_get_attribute(
                             SIM_get_processor(i),
                             "memory_profiling_granularity_log2")));
        
        VCLEAR(bt->data_interval[address_type]);
        VCLEAR(bt->data_stc_interval[address_type]);
        for (i = 0; i < val->u.list.size; i++) {
                attr_value_t *as = val->u.list.vector[i].u.list.vector;
                VADD(bt->data_interval[address_type], create_interval(
                             as[0].u.integer, as[1].u.integer, 0));
                VADD(bt->data_stc_interval[address_type], create_interval(
                             as[0].u.integer, as[1].u.integer, stc_block));
        }

        SIM_flush_all_caches();
        return Sim_Set_Ok;
}

static attr_value_t
get_data_intervals(void *arg, conf_object_t *obj, attr_value_t *idx)
{
	printf("get_data_intervals\n");
        base_trace_t *bt = (base_trace_t *)obj;
        int address_type = (uintptr_t)arg;
        attr_value_t ret;

        ret = SIM_alloc_attr_list(VLEN(bt->data_interval[address_type]));
        VFORI(bt->data_interval[address_type], i) {
                interval_t *iv = &VGET(bt->data_interval[address_type], i);
                ret.u.list.vector[i] = SIM_make_attr_list(
                        2, SIM_make_attr_integer(iv->start),
                        SIM_make_attr_integer(iv->end));
        }
        return ret;
}


/* Cache useful information about each processor. */
static void
cache_cpu_info(base_trace_t *bt)
{
        int num, i;
	/* Cache data for each processor. */
        num = SIM_number_processors();
        bt->cpu = MM_MALLOC(num, cpu_cache_t);
        for (i = 0; i < num; i++) {
                bt->cpu[i].cpu = SIM_get_processor(i);

                bt->cpu[i].info_iface =
                        SIM_c_get_interface(bt->cpu[i].cpu,
                                            PROCESSOR_INFO_INTERFACE);

                bt->cpu[i].pa_digits = (
                        bt->cpu[i].info_iface->get_physical_address_width(
                                bt->cpu[i].cpu) + 3) >> 2;
                bt->cpu[i].va_digits = (
                        bt->cpu[i].info_iface->get_logical_address_width(
                                bt->cpu[i].cpu) + 3) >> 2;

                bt->cpu[i].exception_iface =
                        SIM_c_get_interface(bt->cpu[i].cpu, EXCEPTION_INTERFACE);
                vtsprintf(bt->cpu[i].name, "CPU %2d ", i);
        }

        /* Invent reasonable values for non-cpu devices. */
        bt->device_cpu.va_digits = 16;
        bt->device_cpu.pa_digits = 16;
        strcpy(bt->device_cpu.name, "Device ");
}

static conf_object_t *
base_trace_new_instance(parse_object_t *pa)
{
        base_trace_t *bt = MM_ZALLOC(1, base_trace_t);
        obj_hap_func_t f = (obj_hap_func_t) at_exit_hook;
        int i, j;

        SIM_log_constructor(&bt->log, pa);


        bt->trace_consume = text_tracer; /* text tracing is default */
        bt->trace_instructions = 1;
        bt->trace_data = 1;
        bt->trace_exceptions = 1;
        bt->print_virtual_address = 1;
        bt->print_physical_address = 1;
        bt->print_data = 1;
        bt->print_linear_address = 1;
        bt->print_access_type = 1;
        bt->print_memory_type = 1;
	
        bt->fan = (Fan_info *) malloc(sizeof(Fan_info));
        //bt->redundant = -1;
	bt->redundant = 0;
	bt->fault_injection_instruction = 0;
	bt->fault_injection_core = 0;
	bt->partial_result = 0;
	bt->region = 0;

	bt->redundant_thread = -1;
	bt->redundant_region = 0;
	//bt->index_counter = 0;
	//bt->oldest_index = 0;

	initialize_thread_data(&bt->rr_thread, -1);

	for(i = 0; i < MAX_THREADS; i++)
	{
		initialize_thread_data(&bt->threads[i], i);
                for(j = 0; j < MAX_THREADS; j++)
                {
                    bt->fan->direct_reg_array[i][j] = 0;
                    bt->fan->direct_alu_array[i][j] = 0;
                    bt->fan->direct_mem_array[i][j] = 0;
                    bt->fan->direct_count[i][j] = 0;
                    bt->fan->indirect_reg_array[i][j] = 0;
                    bt->fan->indirect_alu_array[i][j] = 0;
                    bt->fan->indirect_mem_array[i][j] = 0;
                    bt->fan->indirect_count[i][j] = 0;
                }
	}
		
	for(int i = 0; i < 32; i++)
	{
		bt->cores[i] = *MM_ZALLOC(1, core_t);
		bt->cores[i].active = 0;
	}
	bt->memory_store = strmap_new(10000);
	bt->thread_no_map = strmap_new(MAX_THREADS);
        //bt->alt_threshold = 300000000;
        //bt->ust_threshold = 2000000000;
	
	bt->cpu0_active = 0; bt->cpu16_active = 0; 
	bt->cpu1_active = 0; bt->cpu17_active = 0; 
	bt->cpu2_active = 0; bt->cpu18_active = 0; 
	bt->cpu3_active = 0; bt->cpu19_active = 0; 
	bt->cpu4_active = 0; bt->cpu20_active = 0; 
	bt->cpu5_active = 0; bt->cpu21_active = 0; 
	bt->cpu6_active = 0; bt->cpu22_active = 0; 
	bt->cpu7_active = 0; bt->cpu23_active = 0; 
	bt->cpu8_active = 0; bt->cpu24_active = 0; 
	bt->cpu9_active = 0; bt->cpu25_active = 0; 
	bt->cpu10_active = 0; bt->cpu26_active = 0; 
	bt->cpu11_active = 0; bt->cpu27_active = 0; 
	bt->cpu12_active = 0; bt->cpu28_active = 0; 
	bt->cpu13_active = 0; bt->cpu29_active = 0; 
	bt->cpu14_active = 0; bt->cpu30_active = 0; 
	bt->cpu15_active = 0; bt->cpu31_active = 0; 
	
        //if(bt->memory_store == NULL)
	//	write_log("bt->memory_store strmap_new NULL\n");

        for (i = 0; i < NUM_ADDRESS_TYPES; i++) {
                VINIT(bt->data_interval[i]);
                VINIT(bt->data_stc_interval[i]);
        }

        cache_cpu_info(bt);

        SIM_hap_add_callback("Core_At_Exit", f, bt);

	bt->fault_handle = SIM_hap_add_type("Fault_Injection_Hap",
                                      "i",
                                      "val1",
                                      NULL,
                                      "Called when something special "
                                      " happens in my module.",
                                      0);
        return &bt->log.obj;
}

void initialize_thread_data(thread_t *thread, int thread_id)
{
	*thread = *MM_ZALLOC(1, thread_t);
	thread->thread_num = thread_id;
	thread->instr_num = 0;
	thread->instr_count = 0;
        thread->reg_live_area = 0;
	thread->mem_live_area = 0;
	thread->remote_count = 0;
	thread->memory_vul = strmap_new(10000);
	thread->memory_load = strmap_new(10000);
	thread->register_vul = strmap_new(500);
	thread->remote_alu_vul = strmap_new(500);
	thread->remote_reg_vul = strmap_new(500);
	thread->remote_mem_vul = strmap_new(500); 
	thread->remote_count_vul = strmap_new(500); 
	thread->alu_vul = 0;
	thread->reg_vul = 0;
	thread->mem_vul = 0;
        thread->alu_self_vul = 0;
        thread->reg_self_vul = 0;
        thread->mem_self_vul = 0;
	thread->rem_alu_vul = 0;
	thread->rem_reg_vul = 0;
	thread->rem_mem_vul = 0;	
}

static conf_object_t *
trace_new_instance(parse_object_t *pa)
{
        trace_mem_hier_object_t *tmho = MM_ZALLOC(1, trace_mem_hier_object_t);
        SIM_object_constructor(&tmho->obj, pa);
        return &tmho->obj;
}


void
init_local(void)
{
	class_data_t base_funcs;
        conf_class_t *base_class;
        class_data_t trace_funcs;
        conf_class_t *trace_class;
        timing_model_interface_t *timing_iface, *snoop_iface;

        /* first the base class */
        memset(&base_funcs, 0, sizeof(class_data_t));
        base_funcs.new_instance = base_trace_new_instance;
        base_funcs.description =
                "The base class for the trace mode. "
                " This module provides an easy way of generating traces from Simics. "
                "Actions traced are executed instructions, memory accesses and, "
                "occurred exceptions. Traces will by default be printed as text to the "
                "terminal but can also be directed to a file in which case a binary "
                "format is available as well. It is also possible to control what will "
                "be traced by setting a few of the provided attributes.";
	base_funcs.kind = Sim_Class_Kind_Session;
        
        base_class = SIM_register_class("base-trace-mem-hier", &base_funcs);
        
        SIM_register_typed_attribute(
                base_class, "file",
                get_file, NULL, set_file, NULL,
                Sim_Attr_Session, "s|n", NULL,
                "Name of output file that the trace is written to. If the name"
                " ends in <tt>.gz</tt>, the output will be gzipped.");

        SIM_register_typed_attribute(base_class, "raw-mode",
                                     get_raw, NULL,
                                     set_raw, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 for raw "
                                     "output format, and 0 for ascii. Raw output "
                                     "format is only supported when writing to "
                                     "a file.");

        SIM_register_typed_attribute(base_class, "consumer",
                                     get_consumer, NULL,
                                     set_consumer, NULL,
                                     Sim_Attr_Session,
                                     "o|n", NULL,
                                     "Optional consumer object. Must implement "
                                     TRACE_CONSUME_INTERFACE ".");

        SIM_register_typed_attribute(base_class, "enabled",
                                     get_enabled, NULL,
                                     set_enabled, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "tracing, 0 to disable.");

        SIM_register_typed_attribute(base_class, "trace_instructions", 
                                     get_trace_instructions, NULL,
                                     set_trace_instructions, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "instruction tracing, 0 to disable.");

        SIM_register_typed_attribute(base_class, "trace_data", 
                                     get_trace_data, NULL,
                                     set_trace_data, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "tracing of data, 0 to disable.");

        SIM_register_typed_attribute(base_class, "trace_exceptions",
                                     get_trace_exceptions, NULL,
                                     set_trace_exceptions, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "tracing of exceptions, 0 to disable.");

        SIM_register_typed_attribute(base_class, "filter_duplicates",
                                     get_filter_duplicates, NULL,
                                     set_filter_duplicates, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to filter "
                                     "out duplicate trace entries. Useful to filter "
                                     "out multiple steps in looping or repeating "
                                     "instructions.");

        SIM_register_typed_attribute(base_class, "print_virtual_address", 
                                     get_print_virtual_address, NULL,
                                     set_print_virtual_address, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "printing of the virtual address, 0 to disable.");

        SIM_register_typed_attribute(base_class, "print_physical_address", 
                                     get_print_physical_address, NULL,
                                     set_print_physical_address, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "printing of the physical address, 0 to disable.");

        SIM_register_typed_attribute(base_class, "print_linear_address",
                                     get_print_linear_address, NULL,
                                     set_print_linear_address, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "printing of the linear address, 0 to disable.");

        SIM_register_typed_attribute(base_class, "print_access_type", 
                                     get_print_access_type, NULL,
                                     set_print_access_type, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "printing of the memory access type, 0 to disable.");

        SIM_register_typed_attribute(base_class, "print_memory_type", 
                                     get_print_memory_type, NULL,
                                     set_print_memory_type, 0,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "printing of the linear address, 0 to disable.");

        SIM_register_typed_attribute(base_class, "print_data", 
                                     get_print_data, NULL,
                                     set_print_data, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "printing of data and instruction op codes, "
                                     "0 to disable.");

        SIM_register_typed_attribute(
                base_class, "data_intervals_p",
                get_data_intervals, (void *)(uintptr_t)PHYSICAL,
                set_data_intervals, (void *)(uintptr_t)PHYSICAL,
                Sim_Attr_Session, "[[ii]*]", NULL,
                "List of physical address intervals for data tracing."
                " If no intervals are specified, all addresses are traced.");

        SIM_register_typed_attribute(
                base_class, "data_intervals_v",
                get_data_intervals, (void *)(uintptr_t)VIRTUAL,
                set_data_intervals, (void *)(uintptr_t)VIRTUAL,
                Sim_Attr_Session, "[[ii]*]", NULL,
                "List of virtual address intervals for data tracing."
                " If no intervals are specified, all addresses are traced.");

#if defined(TRACE_STATS)
        SIM_register_typed_attribute(base_class, "instruction_records",
                                     get_instruction_records, NULL,
                                     set_instruction_records, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "Instruction records");

        SIM_register_typed_attribute(base_class, "data_records",
                                     get_data_records, NULL,
                                     set_data_records, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "Data records");

        SIM_register_typed_attribute(base_class, "other_records",
                                     get_other_records, NULL,
                                     set_other_records, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "Other records");
#endif   /* TRACE_STATS */

        /* Register new trace class */
        memset(&trace_funcs, 0, sizeof(class_data_t));
        trace_funcs.new_instance = trace_new_instance;
        trace_funcs.description = 
                "This class is defined in the trace module. It is "
                "used by the tracer to listen to memory traffic on "
                "each CPU.";
        
        trace_class = SIM_register_class("trace-mem-hier", &trace_funcs);

        timing_iface = MM_ZALLOC(1, timing_model_interface_t);
        timing_iface->operate = trace_mem_hier_operate;
        SIM_register_interface(trace_class, "timing_model", timing_iface);
        
        snoop_iface = MM_ZALLOC(1, timing_model_interface_t);
        snoop_iface->operate = trace_snoop_operate;
        SIM_register_interface(trace_class, SNOOP_MEMORY_INTERFACE, snoop_iface);
        
	/*static const timing_model_interface_t gc_ifc = {
                .operate = gc_operate
        };
	SIM_register_interface(trace_class, "timing_model", &gc_ifc);*/

        SIM_register_typed_attribute(trace_class, "base_trace_obj",
                                     get_base_trace, NULL,
                                     NULL, NULL,
                                     Sim_Attr_Session,
                                     "o", NULL,
                                     "Base-trace object (read-only)");

        SIM_register_typed_attribute(trace_class, "timing_model",
                                     get_timing_model, NULL,
                                     NULL, NULL,
                                     Sim_Attr_Session,
                                     "o|n", NULL,
                                     "Timing model (read-only)");

        SIM_register_typed_attribute(trace_class, "snoop_device",
                                     get_snoop_device, NULL,
                                     NULL, NULL,
                                     Sim_Attr_Session,
                                     "o|n", NULL,
                                     "Snoop device (read-only)");
	//IO_ register my_task attribute

	SIM_register_typed_attribute(base_class, "partial_result",
                                     get_partial_result, NULL,
                                     set_partial_result, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "redundant_thread",
                                     get_redundant_thread, NULL,
                                     set_redundant_thread, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");
	
	SIM_register_typed_attribute(base_class, "redundant_region",
                                     get_redundant_region, NULL,
                                     set_redundant_region, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

        SIM_register_typed_attribute(base_class, "redundant",
                                     get_redundant, NULL,
                                     set_redundant, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "fault_injection_instruction",
                                     get_fault_injection_instruction, NULL,
                                     set_fault_injection_instruction, NULL,
                                     Sim_Attr_Session,
                                     "s", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "fault_injection_core",
                                     get_fault_injection_core, NULL,
                                     set_fault_injection_core, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu0_active",
                                     get_cpu0_active, NULL,
                                     set_cpu0_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu1_active",
                                     get_cpu1_active, NULL,
                                     set_cpu1_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");
	
	SIM_register_typed_attribute(base_class, "cpu2_active",
                                     get_cpu2_active, NULL,
                                     set_cpu2_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu3_active",
                                     get_cpu3_active, NULL,
                                     set_cpu3_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu4_active",
                                     get_cpu4_active, NULL,
                                     set_cpu4_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu5_active",
                                     get_cpu5_active, NULL,
                                     set_cpu5_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu6_active",
                                     get_cpu6_active, NULL,
                                     set_cpu6_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu7_active",
                                     get_cpu7_active, NULL,
                                     set_cpu7_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu8_active",
                                     get_cpu8_active, NULL,
                                     set_cpu8_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu9_active",
                                     get_cpu9_active, NULL,
                                     set_cpu9_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu10_active",
                                     get_cpu10_active, NULL,
                                     set_cpu10_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu11_active",
                                     get_cpu11_active, NULL,
                                     set_cpu11_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu12_active",
                                     get_cpu12_active, NULL,
                                     set_cpu12_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu13_active",
                                     get_cpu13_active, NULL,
                                     set_cpu13_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu14_active",
                                     get_cpu14_active, NULL,
                                     set_cpu14_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu15_active",
                                     get_cpu15_active, NULL,
                                     set_cpu15_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu16_active",
                                     get_cpu16_active, NULL,
                                     set_cpu16_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu17_active",
                                     get_cpu17_active, NULL,
                                     set_cpu17_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu18_active",
                                     get_cpu18_active, NULL,
                                     set_cpu18_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu19_active",
                                     get_cpu19_active, NULL,
                                     set_cpu19_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu20_active",
                                     get_cpu20_active, NULL,
                                     set_cpu20_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu21_active",
                                     get_cpu21_active, NULL,
                                     set_cpu21_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu22_active",
                                     get_cpu22_active, NULL,
                                     set_cpu22_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu23_active",
                                     get_cpu23_active, NULL,
                                     set_cpu23_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu24_active",
                                     get_cpu24_active, NULL,
                                     set_cpu24_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu25_active",
                                     get_cpu25_active, NULL,
                                     set_cpu25_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu26_active",
                                     get_cpu26_active, NULL,
                                     set_cpu26_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu27_active",
                                     get_cpu27_active, NULL,
                                     set_cpu27_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu28_active",
                                     get_cpu28_active, NULL,
                                     set_cpu28_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu29_active",
                                     get_cpu29_active, NULL,
                                     set_cpu29_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu30_active",
                                     get_cpu30_active, NULL,
                                     set_cpu30_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");

	SIM_register_typed_attribute(base_class, "cpu31_active",
                                     get_cpu31_active, NULL,
                                     set_cpu31_active, NULL,
                                     Sim_Attr_Session,
                                     "i", NULL,
                                     "<tt>1</tt>|<tt>0</tt> Set to 1 to enable "
                                     "my_task, 0 to disable.");
}


typedef struct Pair Pair;

typedef struct Bucket Bucket;

struct Pair {
	char *key;
	char *value;
};

struct Bucket {
	unsigned int count;
	Pair *pairs;
};

struct StrMap {
	unsigned int count;
	Bucket *buckets;
	int size;
	double total_value;
};

static Pair * get_pair(Bucket *bucket, const char *key);
static unsigned long hash(const char *str);

StrMap * strmap_new(unsigned int capacity)
{
	StrMap *map;
	
	map = MM_ZALLOC(1, StrMap);
	if (map == NULL) {
		return NULL;
	}
	map->count = capacity;
	map->buckets = MM_ZALLOC(map->count, Bucket);
	if (map->buckets == NULL) {
		MM_FREE(map);
		return NULL;
	}
	memset(map->buckets, 0, map->count * sizeof(Bucket));
	map->size = 0;
	map->total_value = 0;
	return map;
}

void strmap_delete(StrMap *map)
{
	unsigned int i, j, n, m;
	Bucket *bucket;
	Pair *pair;

	if (map == NULL) {
		return;
	}
	n = map->count;
	bucket = map->buckets;
	i = 0;
	while (i < n) {
		m = bucket->count;
		pair = bucket->pairs;
		j = 0;
		while(j < m) {
			MM_FREE(pair->key);
			MM_FREE(pair->value);
			pair++;
			j++;
		}
		MM_FREE(bucket->pairs);
		bucket++;
		i++;
	}
	MM_FREE(map->buckets);
	MM_FREE(map);
}


int strmap_get(const StrMap *map, const char *key, char *out_buf, unsigned int n_out_buf)
{
	unsigned int index;
	Bucket *bucket;
	Pair *pair;

	if (map == NULL) {
		return 0;
	}
	if (key == NULL) {
		return 0;
	}
	
	index = hash(key) % map->count;
	
	bucket = &(map->buckets[index]);
	if(bucket == NULL){
		return 0;
	}
	pair = get_pair(bucket, key);
	if (pair == NULL) {
		return 0;
	}
	if (out_buf == NULL && n_out_buf == 0) {
		return strlen(pair->value) + 1;
	}
	if (out_buf == NULL) {
		return 0;
	}
	if (strlen(pair->value) >= n_out_buf) {
		return 0;
	}
	strcpy(out_buf, pair->value);
	return 1;
}

int strmap_exists(const StrMap *map, const char *key)
{
	unsigned int index;
	Bucket *bucket;
	Pair *pair;

	if (map == NULL) {
		return 0;
	}
	if (key == NULL) {
		return 0;
	}
	index = hash(key) % map->count;
	bucket = &(map->buckets[index]);
	pair = get_pair(bucket, key);
	if (pair == NULL) {
		return 0;
	}
	return 1;
}

int strmap_put(StrMap *map, const char *key, const char *value)
{
	unsigned int key_len, value_len, index;
	Bucket *bucket;
	Pair *tmp_pairs, *pair;
	char *tmp_value;
	char *new_key, *new_value;

	if (map == NULL) {
		return 0;
	}
	if (key == NULL || value == NULL) {
		return 0;
	}
	key_len = strlen(key);
	value_len = strlen(value);
	/* Get a pointer to the bucket the key string hashes to */
	index = hash(key) % map->count;
	bucket = &(map->buckets[index]);
	/* Check if we can handle insertion by simply replacing
	 * an existing value in a key-value pair in the bucket.
	 */
	if ((pair = get_pair(bucket, key)) != NULL) {
		/* The bucket contains a pair that matches the provided key,
		 * change the value for that pair to the new value.
		 */
		double old_value = atof(pair->value);

		if (strlen(pair->value) < value_len) {
			/* If the new value is larger than the old value, re-allocate
			 * space for the new larger value.
			 */
			//tmp_value = realloc(pair->value, (value_len + 1) * sizeof(char));
			tmp_value = MM_REALLOC(pair->value, (value_len + 1), char);
			if (tmp_value == NULL) {
				return 0;
			}
			pair->value = tmp_value;
		}
		/* Copy the new value into the pair that matches the key */
		
		strcpy(pair->value, value);
		double new_value = atof(pair->value);
                map->total_value = map->total_value - old_value + new_value;
		return 1;
	}
	/*if(map->size >= 10000)
	{	
		return 1;
	}*/
	/* Allocate space for a new key and value */
	//new_key = malloc((key_len + 1) * sizeof(char));
	new_key = MM_ZALLOC((key_len + 1), char);
	
	if (new_key == NULL) {
		return 0;
	}
	//new_value = malloc((value_len + 1) * sizeof(char));
	new_value = MM_ZALLOC((value_len + 1), char);
	if (new_value == NULL) {
		MM_FREE(new_key);
		return 0;
	}
	/* Create a key-value pair */
	if (bucket->count == 0) {
		/* The bucket is empty, lazily allocate space for a single
		 * key-value pair.
		 */
		//bucket->pairs = malloc(sizeof(Pair));
		bucket->pairs = MM_ZALLOC(1,Pair);
		if (bucket->pairs == NULL) {
			MM_FREE(new_key);
			MM_FREE(new_value);
			return 0;
		}
		bucket->count = 1;
	}
	else {
		/* The bucket wasn't empty but no pair existed that matches the provided
		 * key, so create a new key-value pair.
		 */
		//tmp_pairs = realloc(bucket->pairs, (bucket->count + 1) * sizeof(Pair));
		tmp_pairs = MM_REALLOC(bucket->pairs, (bucket->count + 1), Pair);
		if (tmp_pairs == NULL) {
			MM_FREE(new_key);
			MM_FREE(new_value);
			return 0;
		}
		bucket->pairs = tmp_pairs;
		bucket->count++;
	}
	/* Get the last pair in the chain for the bucket */
	pair = &(bucket->pairs[bucket->count - 1]);
	pair->key = new_key;
	pair->value = new_value;
	/* Copy the key and its value into the key-value pair */
	strcpy(pair->key, key);
	strcpy(pair->value, value);
	map->size = map->size + 1;
	if(map->size % 100000 == 0)
		printf("Map size = %d\n", map->size);
	map->total_value = map->total_value + atof(pair->value);
	return 1;
}

/*void handle_capacity(base_trace_t *bt, const char *key)
{	
	StrMap *map = bt->memory_store;
	//printf("Map size = %d\n",map->size);
	int address = atoi(key);
	if(bt->index_counter >= MEMORY_MAP_THRESHOLD)
		bt->index_counter = 0;
	//printf("Counter = %d\n",bt->index_counter);
	bt->index_array[bt->index_counter++] = address;
	if(map->size > MEMORY_MAP_THRESHOLD)
	{
		if(bt->oldest_index >= MEMORY_MAP_THRESHOLD)
			bt->oldest_index = 0;
		//printf("Remove Oldest = %d\n",bt->oldest_index);
		char key_to_del[50];
		sprintf(key_to_del, "%d", bt->index_array[bt->oldest_index++]);
		unsigned int index;
		Bucket *bucket;
		Pair *pair;
		index = hash(key_to_del) % map->count;
		bucket = &(map->buckets[index]);
		if ((pair = get_pair(bucket, key_to_del)) != NULL) {
			MM_FREE(pair->key);
			MM_FREE(pair->value);
			map->size = map->size - 1;
		}else
		{
			printf("pair null\n");
		}
	}
}*/

int strmap_get_count(const StrMap *map)
{
    return map->size;
}

/*int strmap_get_count(const StrMap *map)
{
	unsigned int i, j, n, m;
	unsigned int count;
	Bucket *bucket;
	Pair *pair;

	if (map == NULL) {
		return 0;
	}
	bucket = map->buckets;
	n = map->count;
	i = 0;
	count = 0;
	while (i < n) {
		pair = bucket->pairs;
		m = bucket->count;
		j = 0;
		while (j < m) {
			count++;
			pair++;
			j++;
		}
		bucket++;
		i++;
	}
	return count;
}*/

int strmap_enum(const StrMap *map, strmap_enum_func enum_func, const void *obj)
{
	unsigned int i, j, n, m;
	Bucket *bucket;
	Pair *pair;

	if (map == NULL) {
		return 0;
	}
	if (enum_func == NULL) {
		return 0;
	}
	bucket = map->buckets;
	n = map->count;
	i = 0;
	while (i < n) {
		pair = bucket->pairs;
		m = bucket->count;
		j = 0;
		while (j < m) {
			enum_func(pair->key, pair->value, obj);
			pair++;
			j++;
		}
		bucket++;
		i++;
	}
	return 1;
}

void print_strmap(const StrMap *map, FILE *s)
{
	unsigned int i, j, n, m;
	Bucket *bucket;
	Pair *pair;

	if (map == NULL) {
		printf("MAP EMPTY");		
		return;
	}

	bucket = map->buckets;
	n = map->count;
	i = 0;

	while (i < n) {
		pair = bucket->pairs;
		m = bucket->count;
		j = 0;
		while (j < m) {
			fprintf(s, "%s %s\n",pair->key, pair->value);
			pair++;
			j++;
		}
		bucket++;
		i++;
	}
}

void printf_strmap(const StrMap *map)
{
	unsigned int i, j, n, m;
	Bucket *bucket;
	Pair *pair;

	if (map == NULL) {
		printf("MAP EMPTY");		
		return;
	}

	bucket = map->buckets;
	n = map->count;
	i = 0;
	printf("MAP:\n");
	while (i < n) {
		pair = bucket->pairs;
		m = bucket->count;
		j = 0;
		while (j < m) {
			printf("%s %s\n",pair->key, pair->value);
			pair++;
			j++;
		}
		bucket++;
		i++;
	}
}
/*
 * Returns a pair from the bucket that matches the provided key,
 * or null if no such pair exist.
 */
static Pair * get_pair(Bucket *bucket, const char *key)
{
	unsigned int i, n;
	Pair *pair;
	
	if(bucket == NULL){
		return NULL;
	}
	
	n = bucket->count;
	if (n == 0) {
		return NULL;
	}
	
	pair = bucket->pairs;	
	if(pair == NULL){
		return NULL;
	}
	i = 0;
	while (i < n) {
		if (pair->key != NULL && pair->value != NULL) {
			if (strcmp(pair->key, key) == 0) {
				return pair;
			}
		}
		pair++;
		i++;
	}
	return NULL;
}

/*
 * Returns a hash code for the provided string.
 */
static unsigned long hash(const char *str)
{
	unsigned long hash = 5381;
	int c;

	while (c = *str++) {
		hash = ((hash << 5) + hash) + c;
	}
	return hash;
}

//IO_sum of elements in the map
double sum_strmap(const StrMap *map)
{
	return map->total_value;
}

/*double sum_strmap(const StrMap *map)
{
	unsigned int i, j, n, m;
	Bucket *bucket;
	Pair *pair;
	double sum = 0;
	double value = 0;

	if (map == NULL) {
		printf("MAP EMPTY");		
		return 0;
	}

	bucket = map->buckets;
	if(bucket == NULL)
		return 0;
	n = map->count;
	i = 0;

	while (i < n) {
		pair = bucket->pairs;
		m = bucket->count;
		j = 0;
		while (j < m) {
			value = atof(pair->value);
			sum = sum + value;
			pair++;
			j++;
		}
		bucket++;
		i++;
	}
	return sum;
}*/

