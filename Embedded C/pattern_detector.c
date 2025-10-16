#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h> 
#include <time.h> 
#include <stdlib.h>

#define ARR_SIZE 0xF8 

bool is_detected(uint8_t * a, uint8_t pattern, uint8_t mask);
void bit_arr_generator(uint8_t * bitstream, int size); 
void print_arr(uint8_t * arr, int size); 
void specify_pattern(char * buffer, int buffer_size); 
void parse_pattern(const char * pattern, int * length, uint8_t * value); 

int main(void)
{
    uint8_t currentBit; 
    uint8_t regOut = 0; 
    uint8_t patternMask; 
    uint8_t testArray[ARR_SIZE]; 
    uint8_t patternValue = 0; 
    int patternCounter = 0; 
    int discreteTime = 0; 
    int patternInstances[ARR_SIZE]; 
    char patternString[0x40]; 
    int patternLength = 0; 

    srand(time(NULL)); 
    bit_arr_generator(testArray, ARR_SIZE); 
    specify_pattern(patternString, 0x40); 
    parse_pattern(patternString, &patternLength, &patternValue); 
    patternMask = (1 << patternLength) - 1; 
    
    if (patternLength == 0 || patternLength > 0x08) 
    {
        printf("Error: Invalid pattern. Must be 1-8 bits long.\n");
        return 1; 
    }

    printf("Time | Bit: \n"); 
    for(int bit = 1; bit < ARR_SIZE; ++bit)
    {
        discreteTime++;
        currentBit = testArray[bit]; 
        regOut = is_detected(&currentBit, patternValue, patternMask); 
        printf("%d) %d\n", discreteTime, currentBit); 
        if(regOut == 1)
        {
            printf("User Pattern %s detected at bit index: %d \n", patternString, bit); 
            patternInstances[patternCounter] = bit; 
            ++patternCounter; 
        }
    }
    printf("Total occurrences of pattern %s: %d\n", patternString, patternCounter);
    printf("Locations of each finished occurance: \n");
    for(int index = 0; index < patternCounter; ++index)
    {
        printf("Time: %d,\n", patternInstances[index]); 
    }
    return 0; 
}
void specify_pattern(char * buffer, int buffer_size)
{
    printf("Specify a pattern for the program to detect: \n"); 
    fgets(buffer, buffer_size, stdin); 
    buffer[strcspn(buffer, "\n")] = '\0';
    for (int index = 0; buffer[index] != '\0' ; ++index)
    {
        if(buffer[index] != '0' && buffer[index] != '1')
        {
            printf("Error: Invalid character in pattern. Only '0' and '1' are allowed.\n");
            exit(1); 
        }
    }
}
void bit_arr_generator(uint8_t * bitstream, int size)
{
    for(int generateBit = 0; generateBit < size; ++generateBit)
    {
        bitstream[generateBit] = rand() % 2; 
    }
}
void print_arr(uint8_t * arr, int size)
{
    for(int index = 0; index < size; ++index)
    {
        printf("%d", arr[index]); 
    }
    printf("\n"); 
}   
void parse_pattern(const char * pattern, int * length, uint8_t * value)
{
    *length = 0; 
    *value = 0; 
    for(int index = 0; ; ++index)
    {
       if(pattern[index] != '0' && pattern[index] != '1')
       {
           *length = index; 
           break; 
       }
       *value = (*value << 1) | (pattern[index] - '0');
    }
}


bool is_detected(uint8_t * a, uint8_t pattern, uint8_t mask)
{
    static uint8_t hist = 0; 
    hist = (((hist << 1) | *a) & mask);  
    return (hist == pattern); 
}
