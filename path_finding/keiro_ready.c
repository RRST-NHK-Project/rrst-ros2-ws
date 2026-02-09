//　NHK2026
//  R2
//　経路生成(判別)

#include <stdio.h>

// ブックの種類
enum Type {
    R1,R2,FAKE,EMPTY
};

// ノード生成
typedef struct NODE{
    int row,col;
    enum Type front;
}NODE;

NODE node = {1,1,R2}; //デモ用

void checkfront(enum Type front){
    if(front == R1){
        printf("前方に R1stuff\n");
    }
    else if(front == R2){
        printf("前方に R2stuff\n");
    }
    else if(front == FAKE){
        printf("前方に Fakestuff\n");
    }
    else if(front == EMPTY){
        printf("前方に EmptyArea\n");
    }
    else{
        printf("Error\n");
    }
}



int main(){
    checkfront(node.front);
    printf("処理完了\n");
    return 0;}