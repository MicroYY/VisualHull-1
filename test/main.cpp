#include <iostream>
#include <ctime>
#include <cstdlib>

using namespace std;

int main()
{
    clock_t t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
            }
        }
    }
    t=clock()-t;
    cout<<"Time cost for empty cycle: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    int n=0;
    t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
                n++;
            }
        }
    }
    t=clock()-t;
    cout<<"Time cost for integer inc: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    n=2;
    t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
                n*=2;
            }
        }
    }
    t=clock()-t;
    cout<<"Time cost for integer multiplication: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    n=100;
    t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
                n/=2;
                n+=50;
            }
        }
    }
    t=clock()-t;
    cout<<"Time cost for integer division: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    double d=0;
    t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
                d+=1;
            }
        }
    }
    t=clock()-t;
    cout<<"Time cost for double inc: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    d=1e300;
    t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
                d*=2;
                d-=1e300;
            }
        }
    }
    t=clock()-t;
    cout<<"Time cost for double multiplication: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    d=1e300;
    t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
                d/=2;
                d+=5e299;
            }
        }
    }
    t=clock()-t;
    cout<<"Time cost for double division: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    int *int_ptr=new int;
    t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
                *int_ptr++;
            }
        }
    }
    t=clock()-t;
    cout<<"Time cost for pointer operation "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;


    struct Foo{
        int data[10];
        Foo *prev;
    };
    t=clock();
    Foo *foo_ptr1,*foo_ptr2;    //两个指针构建链表结构便于delete处理
    foo_ptr1=new Foo;
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
                foo_ptr2=new Foo;
                foo_ptr2->prev=foo_ptr1;
                foo_ptr1=foo_ptr2;
            }
        }
    }
    t=clock()-t;
    cout<<"Time cost for creating a object: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){
            for(int k=0;k<300;k++){
                foo_ptr1=foo_ptr2->prev;
                delete foo_ptr2;
                foo_ptr2=foo_ptr1;
            }
        }
    }
    delete foo_ptr2;
    t=clock()-t;
    cout<<"Time cost for deleting a object: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    t=clock();
    for(int i=0;i<300;i++){
        for(int j=0;j<300;j++){     //注意只有二重循环
            foo_ptr1=new Foo[300];  //这里内存泄漏
        }
    }
    t=clock()-t;
    cout<<"Time cost for creating multiple objects: "<<(float(t)/CLOCKS_PER_SEC)<<" sec"<<endl;

    system("pause");
    return 0;
}
