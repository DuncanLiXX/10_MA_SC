/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file DataStack.h
 *@author gonghao
 *@date 2020/04/01
 *@brief 本源文件为数据栈模板类的声明
 *@version
 */

#ifndef DATASTACK_H_
#define DATASTACK_H_

//堆栈数据元结构,T可以是对象也可以是对象指针
template<class T>
struct StackRec {
    T rec;   //数据对象指针
    StackRec* down;    //下面一个数据元
    StackRec* up;    //上一个数据元
    StackRec(T &data):down(nullptr),up(nullptr) {rec = data;}
};

/**
 * @brief 数据存储堆栈
 */
template<class T>
class DataStack {
public://公共接口函数
	DataStack(int max = 0);
	virtual ~DataStack();

	bool pop(T &data);       //弹出栈顶数据
	void pop();             //弹出栈顶数据并丢弃
	void pop2();			//弹出栈顶元素但不清除或释放空间
	bool cur(T &data);     //获取栈顶数据，但是不弹出
	bool bottom(T &data);  //获取栈底数据，但不弹出
	bool push(T &data);  //入栈操作
	void empty();    //清空栈内数据
	int size();      //数据个数
	void set_max_size(int max);   // 设置最大容量
	int max_size() const{return m_n_max_count;}      //获取堆栈最大容量

	StackRec<T> *top()const {return m_p_top;}   //栈顶数据元指针
	StackRec<T> *bottom()const {return m_p_bottom;}   //栈底数据元指针

	DataStack<T> &operator=(const DataStack<T> &st);   //重载赋值运算符

private://私有接口函数
//	virtual void freeItem( T *data );   //释放数据元内存空间

	//区分指针和对象两种处理
	void DeleteData(std::true_type, T data){
		printf("delete data1\n");
		delete data;
		printf("delete data2\n");
	}
	void DeleteData(std::false_type, T data){}
private://私有成员变量
	StackRec<T> *m_p_top;     //栈顶指针
	StackRec<T> *m_p_bottom;  //栈底指针
	int  m_n_count;       //栈内数据数量
	int m_n_max_count;     //栈内容量，为0则表示无限制，超过此值时自动丢弃最早的数据
};

/**
 * @brief 数据堆栈类构造函数
 * @param bool bPointer:标志数据T是否为指针型
 */
template<class T>
DataStack<T>::DataStack(int max): m_p_top(nullptr),m_p_bottom(nullptr),m_n_count(0),m_n_max_count(max) {
	// TODO Auto-generated constructor stub
}

/**
 * @brief 数据堆栈类析构函数
 */
template<class T>
DataStack<T>::~DataStack() {
	// TODO Auto-generated destructor stub
	empty();
}


/**
 * @brief 入栈操作函数，即将数据放入堆栈顶部
 * @param T &data: 需要放入堆栈的数据
 * @return true--成功， false--失败
 */
template<class T>
bool DataStack<T>::push(T &data)
{
	bool res = true;
    StackRec<T> *p = new StackRec<T>(data);

    if( p )
    {
    //    p->rec = data;
        p->down = m_p_top;
        if(m_p_top)
        	m_p_top->up = p;
        m_p_top = p;

        if( m_n_count == 0 ) m_p_bottom = p;


        if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //栈内数据量超限，则自动丢弃最早的数据
        	StackRec<T> *tmp = m_p_bottom;
        	m_p_bottom = tmp->up;
        	if(m_p_bottom)
        		m_p_bottom->down = nullptr;
        	delete tmp;
        }else{
        	m_n_count++;
        }
    }
    else{
    	res = false;
    }
    return res;
}

/**
 * @brief 弹出栈顶数据
 * @param data : 栈顶数据
 * @return true--操作成功   false--操作失败，无数据可取
 */
template<class T>
bool DataStack<T>::pop(T &data)
{
	if( m_n_count <= 0 ) return false;

	printf("data stack pop&data 1\n");

    data = m_p_top->rec;
    StackRec<T> *p = m_p_top->down;
    if(p != nullptr)
    	p->up = nullptr;
    delete m_p_top;
    m_p_top = p;
    m_n_count--;

    if( m_n_count == 0 ) m_p_bottom = nullptr;

    printf("data stack pop&data 2\n");

    return true;
}

/**
 * @brief 弹出栈顶数据并丢弃
 * @return
 */
template<class T>
void DataStack<T>::pop()
{
    if( m_n_count <= 0 ) return;

    T data = m_p_top->rec;
    StackRec<T> *p = m_p_top->down;
    if(p != nullptr)
    	p->up = nullptr;
    DeleteData(std::is_pointer<T>(), data);
    delete m_p_top;
    m_p_top = p;
    m_n_count--;

    if( m_n_count == 0 ) m_p_bottom = nullptr;
}

/**
 * @brief 弹出栈顶元素但不清除或释放空间
 * @return
 */
template<class T>
void DataStack<T>::pop2()
{
    if( m_n_count <= 0 ) return;

    StackRec<T> *p = m_p_top->down;
    if(p != nullptr)
    	p->up = nullptr;
    m_p_top = p;
    m_n_count--;

    if( m_n_count == 0 ) m_p_bottom = nullptr;
}


/**
 * @brief 获取栈顶数据，但是不弹出
 * @param data : 栈顶数据的引用
 * @return false：无数据  true--成功
 */
template<class T>
bool DataStack<T>::cur(T &data)
{
	if(m_n_count <= 0) return false;
	data = m_p_top->rec;
    return true;
}

/**
 * @brief 获取栈底数据，但不弹出
 * @param data : 栈底数据的引用
 * @return false：无数据  true--成功
 */
template<class T>
bool DataStack<T>::bottom(T &data){
	if(m_n_count <= 0) return false;
	data = m_p_bottom->rec;
    return true;
}

/**
 * @brief 当前堆栈数据个数
 */
template<class T>
int DataStack<T>::size(){
	return m_n_count;
}


/**
 * @brief 清空栈内数据
 */
template<class T>
void DataStack<T>::empty()
{
    while( m_n_count ) {
    	pop();

    }
}



/**
 * @brief 重载赋值运算符
 * @param list
 * @return
 */
template<typename T>
DataStack<T> &DataStack<T>::operator=(const DataStack<T> &st){
	if(&st == this)
		return *this;
	empty();

	this->set_max_size(st.max_size());   //设置最大容量

	StackRec<T> *node = st.bottom();
	while(node != nullptr){
		this->push(node->rec);

		node = node->up;
	}

	return *this;
}


/**
 * @brief 设置当前堆栈最大容量
 */
template<class T>
void DataStack<T>::set_max_size(int max){
	m_n_max_count = max;

	//删除超过容量的数据
	StackRec<T> *tmp = nullptr;
	while(m_n_max_count > 0 && m_n_count > m_n_max_count){
       	tmp = m_p_bottom;
		m_p_bottom = tmp->up;
		if(m_p_bottom)
			m_p_bottom->down = nullptr;
		delete tmp;
		m_n_count--;
	}
}


#endif /* DATASTACK_H_ */
