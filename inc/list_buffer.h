/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file list_buffer.h
 *@author gonghao
 *@date 2020/03/23
 *@brief 本头文件包含链表类声明
 *@version
 */

#ifndef LISTBUFFER_H_
#define LISTBUFFER_H_

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include<type_traits>

using namespace std;


//链表节点结构体
template<typename T>
class ListNode{
public:
	ListNode();
	~ListNode();
	T data;    //数据指针
	ListNode *next;   //下一个节点的指针
	ListNode *pre;    //前一个节点的指针
//	static int new_count;   //for test用于测试内存泄漏
};


template<typename T>
ListNode<T>::ListNode(){
//	new_count++;
}

template<typename T>
ListNode<T>::~ListNode(){
//	new_count--;
}


/**
 * @brief 双向链表类，存储数据可以兼容对象和对象指针
 *
 */
template<typename T>
class ListBuffer {
public:

	ListBuffer(int max = 0);
	ListBuffer(const ListBuffer<T> &list);
	virtual ~ListBuffer();


	bool Append(const T &data);          //在尾部添加
	bool Append(ListNode<T> *node); 		//在尾部添加节点node
	bool InsertBefore(const T &data, ListNode<T> *pos);  //在pos之前插入节点data
	bool InsertAfter(const T &data, ListNode<T> *pos);   //在pos之后插入节点data
	bool Delete(ListNode<T> *node, bool mutex_test = true);  //删除指定节点node
	bool Delete(const T &data);    //删除数据为data的节点
	ListNode<T>* RemoveNode(ListNode<T> *node);  //从链表中移除node节点，但是不删除
	ListNode<T>* RemoveHead();     //移除头结点
	bool Clear();       //清空整个链表
	bool ClearBefore(ListNode<T> *node);  //删除node以前的所有缓冲数据，包括node本身
	int GetLength(){return m_n_count;}  //获取链表长度
	bool IsEmpty(){return (m_ptr_head==NULL)?true:false;}   //链表是否为空
	bool HasData(const T &data);		//队列中是否存在数据data

	int GetMaxSize() const{return m_n_max_count;}      //获取链表最大容量

	ListNode<T> *HeadNode() const {return m_ptr_head;}    //获取头指针节点
	ListNode<T> *TailNode() const {return m_ptr_tail;}    //获取尾指针节点

	ListBuffer<T> &operator=(const ListBuffer<T> &list);   //重载赋值运算符

private:

	//区分指针和对象两种处理
	void DeleteData(std::true_type, T data){delete data;}
	void DeleteData(std::false_type, T data){}

//私有成员变量
private:
	ListNode<T> *m_ptr_head;   //头指针
	ListNode<T> *m_ptr_tail;   //尾指针
	int m_n_count;            //节点个数
//	bool m_b_pointer;         //节点数据类型T是否为指针
	pthread_mutex_t m_mutex;  //用于访问互斥
	int m_n_max_count;     //栈内容量，为0则表示无限制，超过此值时自动丢弃最早的数据
};


/**
 * @brief 双向链表构造函数
 * @param 无
 * @return 无
 */
template<typename T>
ListBuffer<T>::ListBuffer(int max){
	// TODO Auto-generated constructor stub
	m_ptr_head = nullptr;
	m_ptr_tail = nullptr;
	m_n_count = 0;
	this->m_n_max_count = max;
	pthread_mutex_init(&m_mutex, NULL);


}

/**
 * @brief 拷贝构造函数
 * @param list ： 被拷贝对象
 */
template<typename T>
ListBuffer<T>::ListBuffer(const ListBuffer<T> &list){
	// TODO Auto-generated constructor stub
	m_ptr_head = nullptr;
	m_ptr_tail = nullptr;
	m_n_count = 0;
	m_n_max_count = list.GetMaxSize();
	pthread_mutex_init(&m_mutex, NULL);

	ListNode<T> *node = list.HeadNode();
	while(node != nullptr){
		this->Append(node->data);

		node = node->next;
	}

}

/**
 * @brief 双向链表析构函数，释放空间
 * @param 无
 * @return 无
 */
template<typename T>
ListBuffer<T>::~ListBuffer() {
	// TODO Auto-generated destructor stub
	Clear();
	m_ptr_head = nullptr;
	m_ptr_tail = nullptr;
	m_n_count = 0;
	pthread_mutex_destroy(&m_mutex);
}

/**
 * @brief 重载赋值运算符
 * @param list
 * @return
 */
template<typename T>
ListBuffer<T> &ListBuffer<T>::operator=(const ListBuffer<T> &list){
	if(&list == this)
		return *this;
	Clear();

	m_n_max_count = list.GetMaxSize();
	ListNode<T> *node = list.HeadNode();
	while(node != nullptr){
		this->Append(node->data);

		node = node->next;
	}

	return *this;
}

/**
 * @brief 在双向链表的尾部增加节点
 * @param node : 节点数据
 * @return true:成功   false:失败
 */
template<typename T>
bool ListBuffer<T>::Append(ListNode<T> *node){

	if(node == nullptr)
		return true;  //node为空

	node->next = nullptr;
	node->pre = nullptr;

	pthread_mutex_lock(&m_mutex);

	if(m_n_count == 0){//空链表
		m_ptr_head = m_ptr_tail = node;
	}
	else{//尾部添加
		m_ptr_tail->next = node;
		node->pre = m_ptr_tail;
		m_ptr_tail = node;
	}

	if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //链表数据量超限，则自动丢弃最早的数据，此时数据数量不用自增
		this->Delete(this->m_ptr_head, false);
	}
	m_n_count++;

	pthread_mutex_unlock(&m_mutex);
	return true;
}

/**
 * @brief 在双向链表的尾部增加节点
 * @param T &data: 需要加入新节点的数据
 * @return true:成功   false:失败
 */
template<typename T>
bool ListBuffer<T>::Append(const T &data){

	ListNode<T> *new_node = new ListNode<T>();
	if(new_node == nullptr)
		return false;  //分配空间失败
	new_node->data = data;
	new_node->next = nullptr;
	new_node->pre = nullptr;

	pthread_mutex_lock(&m_mutex);

	if(m_n_count == 0){//空链表
		m_ptr_head = m_ptr_tail = new_node;
	}
	else{//尾部添加
		m_ptr_tail->next = new_node;
		new_node->pre = m_ptr_tail;
		m_ptr_tail = new_node;
	}

	if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //链表数据量超限，则自动丢弃最早的数据，此时数据数量不用自增
		this->Delete(this->m_ptr_head, false);
	}
	m_n_count++;

	pthread_mutex_unlock(&m_mutex);
	return true;
}

/**
 * @brief 在pos之前插入节点data
 * @param data : 待插入的数据
 * @param pos ：插入位置
 * @return
 */
template<typename T>
bool ListBuffer<T>::InsertBefore(const T &data, ListNode<T> *pos){
	if(pos == nullptr)
		return false;

	//创建新节点
	ListNode<T> *new_node = new ListNode<T>();
	if(new_node == nullptr)
		return false;  //分配空间失败
	new_node->data = data;
	new_node->next = nullptr;
	new_node->pre = nullptr;

	pthread_mutex_lock(&m_mutex);

	new_node->next = pos;
	new_node->pre = pos->pre;
	pos->pre = new_node;
	if(pos == m_ptr_head){//头部插入
		m_ptr_head = new_node;
	}else{
		new_node->pre->next = new_node;
	}

	if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //链表数据量超限，则自动丢弃最早的数据，此时数据数量不用自增
		this->Delete(this->m_ptr_head, false);
	}
	m_n_count++;

	pthread_mutex_unlock(&m_mutex);
	return true;
}

/**
 * @brief 在pos之后插入节点data
 * @param data : 待插入的数据
 * @param pos ：插入位置
 * @return
 */
template<typename T>
bool ListBuffer<T>::InsertAfter(const T &data, ListNode<T> *pos){
	if(pos == nullptr)
		return false;

	//创建新节点
	ListNode<T> *new_node = new ListNode<T>();
	if(new_node == nullptr)
		return false;  //分配空间失败
	new_node->data = data;
	new_node->next = nullptr;
	new_node->pre = nullptr;

	pthread_mutex_lock(&m_mutex);

	new_node->next = pos->next;
	new_node->pre = pos;
	pos->next = new_node;
	if(pos == m_ptr_tail){//尾部插入
		m_ptr_tail = new_node;
	}else{
		new_node->next->pre = new_node;
	}

	if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //链表数据量超限，则自动丢弃最早的数据，此时数据数量不用自增
		this->Delete(this->m_ptr_head, false);
	}
	m_n_count++;

	pthread_mutex_unlock(&m_mutex);

	return true;
}

/**
 * @brief 从链表中移除node节点，但是不删除
 * @param node : 待移除的节点指针
 * @return
 */
template<typename T>
ListNode<T>* ListBuffer<T>::RemoveNode(ListNode<T> *node){
	if(node == nullptr)
		return nullptr;

	if(node == m_ptr_head){
		if(m_n_count == 1){//移除唯一节点
			m_ptr_head = m_ptr_tail = nullptr;
			m_n_count = 0;
			return node;
		}
		m_ptr_head = node->next;
		node->next->pre = nullptr;
	}else if(node == m_ptr_tail){
		m_ptr_tail = node->pre;
		node->pre->next = nullptr;
	}else{
		node->pre->next = node->next;
		node->next->pre = node->pre;
	}
	m_n_count--;
	node->next = nullptr;
	node->pre = nullptr;
	return node;

}

template<typename T>
ListNode<T>* ListBuffer<T>::RemoveHead(){

	return RemoveNode(m_ptr_head);
}


/**
 * @brief 在双向链表中删除节点
 * @param T &data: 待删除节点的数据
 * @return true:成功   false:失败
 */
template<typename T>
bool ListBuffer<T>::Delete(const T &data){

	if(m_ptr_head == nullptr || m_n_count == 0)
		return true;

	pthread_mutex_lock(&m_mutex);

	ListNode<T> *node = m_ptr_head;
	while(node != nullptr){
		if(node->data == data)
			break;
		node = node->next;
	}

	if(node == nullptr){
		pthread_mutex_unlock(&m_mutex);
		return false;   //没找到要删除的节点
	}

	if(m_n_count == 1){//删除唯一节点
		m_ptr_head = m_ptr_tail = nullptr;
	}
	else if(node == m_ptr_head){//删除头结点
		m_ptr_head = node->next;
		node->next->pre = nullptr;
	}
	else if(node == m_ptr_tail){ //删除尾结点
		m_ptr_tail = node->pre;
		node->pre->next = nullptr;
	}
	else { //删除中间节点
		node->pre->next = node->next;
		node->next->pre = node->pre;
	}

	this->DeleteData(std::is_pointer<T>(), node->data);

//	if(m_b_pointer)
//		this->m_p_del_fun(node->data);
	delete node;
	m_n_count--;

	pthread_mutex_unlock(&m_mutex);
	return true;
}


/**
 * @brief 在双向链表中删除节点
 * @param ListNode<T> * node: 待删除节点指针
 * @param mutex_test : 是否需要检测mutex
 * @return true:成功   false:失败
 */
template<typename T>
bool ListBuffer<T>::Delete(ListNode<T> * node, bool mutex_test){

	if(node == nullptr || m_n_count == 0)
		return true;

	if(mutex_test)
		pthread_mutex_lock(&m_mutex);

	if(m_n_count == 1 && node == m_ptr_head){//删除唯一节点
		m_ptr_head = m_ptr_tail = nullptr;
	}
	else if(node == m_ptr_head){//删除头结点
		m_ptr_head = node->next;
		node->next->pre = nullptr;
	}
	else if(node == m_ptr_tail){ //删除尾结点
		m_ptr_tail = node->pre;
		node->pre->next = nullptr;
	}
	else { //删除中间节点
		node->pre->next = node->next;
		node->next->pre = node->pre;
	}

	this->DeleteData(std::is_pointer<T>(), node->data);
//	if(m_b_pointer)
//		this->m_p_del_fun(node->data);
	delete node;
	m_n_count--;

	if(mutex_test)
		pthread_mutex_unlock(&m_mutex);
	return true;
}


/**
 * @brief 清空链表
 * @param 无
 * @return true:成功   false:失败
 */
template<typename T>
bool ListBuffer<T>::Clear(){
	if(m_n_count == 0)
		return true;

	pthread_mutex_lock(&m_mutex);

	ListNode<T> *node = m_ptr_head;
	ListNode<T> *node_del = nullptr;
	while(node != nullptr){
		node_del = node;
		node = node->next;
		this->DeleteData(std::is_pointer<T>(), node_del->data);
//		if(m_b_pointer){
//			this->m_p_del_fun(node_del->data);
//		}
		delete node_del;
	}

	m_ptr_head = m_ptr_tail = nullptr;
	m_n_count = 0;

	pthread_mutex_unlock(&m_mutex);
	return true;
}

/**
 * @brief 删除node以前的所有缓冲数据，包括node本身
 * @param node : 指定的缓冲数据节点
 * @return true:成功   false:失败
 */
template<typename T>
bool ListBuffer<T>::ClearBefore(ListNode<T> *node){
	if(m_n_count == 0)
		return true;

	pthread_mutex_lock(&m_mutex);

	ListNode<T> *node_cur = m_ptr_head;
	ListNode<T> *node_del = nullptr;
	bool bb = false;   //跳出标志
	while(node_cur != nullptr){
		node_del = node_cur;
		node_cur = node_cur->next;
		if(node_del == node)
			bb = true;

		if(node_cur != nullptr)
			node_cur->pre = nullptr;

		this->DeleteData(std::is_pointer<T>(), node_del->data);

		delete node_del;

		m_n_count--;
		if(bb)
			break;
	}

	m_ptr_head = node_cur;
	if(m_n_count == 0)
		this->m_ptr_tail = nullptr;


	pthread_mutex_unlock(&m_mutex);
	return true;
}

/**
 * @brief 队列中是否存在数据data
 * @param data
 * @return true--存在相同数据   false--不存在相同数据
 */
template<typename T>
bool ListBuffer<T>::HasData(const T &data){
	if(m_n_count == 0)
		return false;

	pthread_mutex_lock(&m_mutex);

	ListNode<T> *node = m_ptr_head;
	while(node != nullptr){
		if(node->data == data)
			break;
		node = node->next;
	}
	pthread_mutex_unlock(&m_mutex);
	if(node == nullptr)
		return false;
	return true;
}


#endif /* LISTBUFFER_H_ */
