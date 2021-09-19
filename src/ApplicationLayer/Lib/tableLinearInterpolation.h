#pragma once

inline float tableLinearInterpolation(float in, const float* table_in, const float* table_out, int table_size){
    if(table_size == 1){
        return table_out[0];
    }
    if(in < table_in[0]){
        return table_out[0];
    } 
    if(in > table_in[table_size - 1]){
        return table_out[table_size-1];
    } 
    
    int head = 0;
    int tail = table_size;
    int index = 0;
    
    while(head <= tail){
        index = (head + tail) / 2;
        if( table_in[index] < in && table_in[index+1] > in  ){
            break;
        } 
        else if(table_in[index] < in){
            head = index + 1;
        }
        else{
            tail = index - 1;
        }                
    }

    float rasio = (in - table_in[index]) /(table_in[index+1] - table_in[index]);
    float out = table_out[index] + (table_out[index+1] - table_out[index]) * rasio;
    return out;
}

float tableLinearInterpolation(float in, float in_min, float in_max, float in_delta, const float* table_out, int table_size){
    if(table_size == 1){
        return table_out[0];
    }
    if(in < in_min){
        return table_out[0];
    } 
    if(in > in_max){
        return table_out[table_size-1];
    } 
       
    float index =  (in - in_min) / (in_max - in_min) * (table_size -1);    
    float ratio = index - (int)index;
    float out = table_out[(int)index] + (table_out[(int)index+1] - table_out[(int)index]) * ratio;
    return out;
}
