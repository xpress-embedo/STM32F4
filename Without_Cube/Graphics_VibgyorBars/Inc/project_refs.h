
#ifndef PROJECT_REFS_H_
#define PROJECT_REFS_H_

/* Register bit manipulation macros */

#define SET_VALUE(reg, val)                 ((reg) = (val))
#define GET_VALUE(reg)                      ((reg))
#define SET_BIT(reg,pos)                    ((reg) |=  (1U << (pos)))
#define CLR_BIT(reg,pos)                    ((reg) &= ~(1U << (pos)))
#define READ_BIT(reg,pos)                   ((reg) &   (1U << (pos)))
#define REG_CLR_VAL(reg,clrmask,pos)        ((reg) &= ~((clrmask) << (pos)))
#define REG_SET_VAL(reg,val,setmask,pos)    do {\
                                                  REG_CLR_VAL(reg,setmask,pos);\
                                                  ((reg) |= ((val) << (pos))); \
                                                }while(0)

#define REG_READ_VAL(reg,rdmask,pos)        ((GET_VALUE(reg) >> (pos)) & (rdmask))


#endif /* PROJECT_REFS_H_ */
