https://www.embedded.com/disabling-interrupts/

Blog
Disabling Interrupts
 March 22, 2010 Jack Ganssle
I wish we lived in an atomic world.

No, I am not yearning for an Iranian bomb. Rather, I am referring tothe fact that unavoidable non-atomic accesses to shared resourcescauses much grief and poor code.

I read a lot of code, and find much that handles non-atomic accessesin this manner:

long global_var;
void do_something(void)
{

// Handle a non-atomic access to "global_var"
#pragma disable interrupts somehow
// Do something non-atomic to global_var
#pragma enable interrupts somehow
}

This construct suffers from a number of problems, not the least ofwhich is that it's not generally reuseable. If the function is calledfrom some place with interrupts off, it returns with them on,disrupting the system's context.

Usual solutions involve saving and restoring the interrupt state.But that, too, is fraught with peril. Optimizers are very aggressivetoday, and in some cases can reorder statement execution to the pointwhere interrupts aren't disabled at the right point. The result: allthat atomic-work may not work.

I've asked several compiler vendors for their take, since theyunderstand the optimizations the compilers do better than anyone. Themost interesting and complete response came from Greg Davis of GreenHills Software, and he has graciously allowed me to reprint it here:

“What we recommend for Green Hills customers is to use intrinsicfunctions for disabling and restoring interrupts. What this looks likeis:

#include
int global_var;
void foo(void)
{
    // Disable interrupts and return "key"
    // that expresses current interrupt state unsignedint key = __DIR();

    // Code that handles global_var in anon-atomic way

    // Restore interrupts to state expressed by"key"
    __RIR(key);
}"

These Green Hills intrinsics for __DIR() and __RIR() generate different assembly code depending on the architecture and CPU that you are compiling for, but their interface is the same. The compiler considers the system-instructions that these intrinsics generate to be non-swappable, so the code that manipulates global_var will not be swapped across them.

With GNU, people tend to prefer to use inline assembly. These assembly statements are typically embedded in inline functions or macros with GNU statement expressions. An implementation of something like this on an ARM7TDMI might look like:

static inline unsigned int
disable_interrupts_reentrant(void)
{
    unsigned int ret;
    asm volatile(
        "mrs %0,cpsrn"
        "orr r1,%0,192n"
        "msr cpsr_cxsf,r1n"
        : /* output */ "=r" (ret)
        : /* input */
        : /* clobbers */ "r1", "memory"
        );
    return ret;
}

static inline void restore_interrupts(unsigned int state)
{
    asm volatile(
        "and r1,%0,192n"
        "mrs r0,cpsrn"
        "bic r0,r0,192n"
        "orr r0,r0,r1n"
        "msr cpsr_cxsf,r0n"
        : /* output */
        : /* input */ "r" (state)
        : /* clobbers */ "r0", "r1","memory"
        );
}

int global_var;
void foo(void)
{
    unsigned int key = disable_interrupts_reentrant();

    // Code that handles global_var in a non-atomicway

    restore_interrupts(key);
}

At least to my understanding, the combination of the declaring the assembly to be volatile and putting the “memory” in the clobbers list should ensure that memory accesses in the critical section stay in thecritical section.

Both of the above approaches involve compiler-specific extensions.The best approach I'm aware of that isn't compiler specific is to move the code into another file so it just looks like a function call to the compiler:

extern unsigned int disable_interrupts_reentrant(void);
extern void restore_interrupts(unsigned int state);
int global_var;
void foo(void)
{
    unsigned int key = disable_interrupts_reentrant();

    // Code that handles global_var in a non-atomic way

    restore_interrupts(key);
}

and then to define the functions in a separate assembly file. The exact assembly syntax may vary between implementations, but it may look something like this on a traditional UNIX-style assembler.

    .text
    .globl disable_interrupts_reentrant
disable_interrupts_reentrant:
    ; Inputs: none
    ; Outputs: r0 (return register) contains a key forthe
    ; current interrupt state
    mrs r0, cpsr
    orr r1, r0, 192
    msr cpsr_cxsf, r1
    bx lr
    .type disable_interrupts_reentrant, @function
    .size disable_interrupts_reentrant, .-
    disable_interrupts_reentrant

    .globl restore_interrupts
restore_interrupts:
    ; Inputs: r0: prior interrupt state ;
    Outputs: None
    and r1, r0, 192
    mrs r0, cpsr
    bic r0, r0, 192
    orr r0, r0, r1
    msr cpsr_cxsf,r0
    bx lr
    .type restore_interrupts, @function
    .size restore_interrupts, .-restore_interrupts

Since compilers need to assume that external functions read and write all global variables, there's no chance for the code that handles global_var to fall outside of the critical section.”


Thanks, Greg, for the insight. I hope this information is useful tofolks.


Jack G. Ganssle is a lecturer and consultant on embedded development issues. He conducts seminars on embedded systems and helps companies with their embedded challenges