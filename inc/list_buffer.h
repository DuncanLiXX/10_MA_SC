/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file list_buffer.h
 *@author gonghao
 *@date 2020/03/23
 *@brief ��ͷ�ļ���������������
 *@version
 */

#ifndef LISTBUFFER_H_
#define LISTBUFFER_H_

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include<type_traits>

using namespace std;


//����ڵ�ṹ��
template<typename T>
class ListNode{
public:
	ListNode();
	~ListNode();
	T data;    //����ָ��
	ListNode *next;   //��һ���ڵ��ָ��
	ListNode *pre;    //ǰһ���ڵ��ָ��
//	static int new_count;   //for test���ڲ����ڴ�й©
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
 * @brief ˫�������࣬�洢���ݿ��Լ��ݶ���Ͷ���ָ��
 *
 */
template<typename T>
class ListBuffer {
public:

	ListBuffer(int max = 0);
	ListBuffer(const ListBuffer<T> &list);
	virtual ~ListBuffer();


	bool Append(const T &data);          //��β�����
	bool Append(ListNode<T> *node); 		//��β����ӽڵ�node
	bool InsertBefore(const T &data, ListNode<T> *pos);  //��pos֮ǰ����ڵ�data
	bool InsertAfter(const T &data, ListNode<T> *pos);   //��pos֮�����ڵ�data
	bool Delete(ListNode<T> *node, bool mutex_test = true);  //ɾ��ָ���ڵ�node
	bool Delete(const T &data);    //ɾ������Ϊdata�Ľڵ�
	ListNode<T>* RemoveNode(ListNode<T> *node);  //���������Ƴ�node�ڵ㣬���ǲ�ɾ��
	ListNode<T>* RemoveHead();     //�Ƴ�ͷ���
	bool Clear();       //�����������
	bool ClearBefore(ListNode<T> *node);  //ɾ��node��ǰ�����л������ݣ�����node����
	int GetLength(){return m_n_count;}  //��ȡ������
	bool IsEmpty(){return (m_ptr_head==NULL)?true:false;}   //�����Ƿ�Ϊ��
	bool HasData(const T &data);		//�������Ƿ��������data

	int GetMaxSize() const{return m_n_max_count;}      //��ȡ�����������

	ListNode<T> *HeadNode() const {return m_ptr_head;}    //��ȡͷָ��ڵ�
	ListNode<T> *TailNode() const {return m_ptr_tail;}    //��ȡβָ��ڵ�

	ListBuffer<T> &operator=(const ListBuffer<T> &list);   //���ظ�ֵ�����

private:

	//����ָ��Ͷ������ִ���
	void DeleteData(std::true_type, T data){delete data;}
	void DeleteData(std::false_type, T data){}

//˽�г�Ա����
private:
	ListNode<T> *m_ptr_head;   //ͷָ��
	ListNode<T> *m_ptr_tail;   //βָ��
	int m_n_count;            //�ڵ����
//	bool m_b_pointer;         //�ڵ���������T�Ƿ�Ϊָ��
	pthread_mutex_t m_mutex;  //���ڷ��ʻ���
	int m_n_max_count;     //ջ��������Ϊ0���ʾ�����ƣ�������ֵʱ�Զ��������������
};


/**
 * @brief ˫�������캯��
 * @param ��
 * @return ��
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
 * @brief �������캯��
 * @param list �� ����������
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
 * @brief ˫�����������������ͷſռ�
 * @param ��
 * @return ��
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
 * @brief ���ظ�ֵ�����
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
 * @brief ��˫�������β�����ӽڵ�
 * @param node : �ڵ�����
 * @return true:�ɹ�   false:ʧ��
 */
template<typename T>
bool ListBuffer<T>::Append(ListNode<T> *node){

	if(node == nullptr)
		return true;  //nodeΪ��

	node->next = nullptr;
	node->pre = nullptr;

	pthread_mutex_lock(&m_mutex);

	if(m_n_count == 0){//������
		m_ptr_head = m_ptr_tail = node;
	}
	else{//β�����
		m_ptr_tail->next = node;
		node->pre = m_ptr_tail;
		m_ptr_tail = node;
	}

	if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //�������������ޣ����Զ�������������ݣ���ʱ����������������
		this->Delete(this->m_ptr_head, false);
	}
	m_n_count++;

	pthread_mutex_unlock(&m_mutex);
	return true;
}

/**
 * @brief ��˫�������β�����ӽڵ�
 * @param T &data: ��Ҫ�����½ڵ������
 * @return true:�ɹ�   false:ʧ��
 */
template<typename T>
bool ListBuffer<T>::Append(const T &data){

	ListNode<T> *new_node = new ListNode<T>();
	if(new_node == nullptr)
		return false;  //����ռ�ʧ��
	new_node->data = data;
	new_node->next = nullptr;
	new_node->pre = nullptr;

	pthread_mutex_lock(&m_mutex);

	if(m_n_count == 0){//������
		m_ptr_head = m_ptr_tail = new_node;
	}
	else{//β�����
		m_ptr_tail->next = new_node;
		new_node->pre = m_ptr_tail;
		m_ptr_tail = new_node;
	}

	if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //�������������ޣ����Զ�������������ݣ���ʱ����������������
		this->Delete(this->m_ptr_head, false);
	}
	m_n_count++;

	pthread_mutex_unlock(&m_mutex);
	return true;
}

/**
 * @brief ��pos֮ǰ����ڵ�data
 * @param data : �����������
 * @param pos ������λ��
 * @return
 */
template<typename T>
bool ListBuffer<T>::InsertBefore(const T &data, ListNode<T> *pos){
	if(pos == nullptr)
		return false;

	//�����½ڵ�
	ListNode<T> *new_node = new ListNode<T>();
	if(new_node == nullptr)
		return false;  //����ռ�ʧ��
	new_node->data = data;
	new_node->next = nullptr;
	new_node->pre = nullptr;

	pthread_mutex_lock(&m_mutex);

	new_node->next = pos;
	new_node->pre = pos->pre;
	pos->pre = new_node;
	if(pos == m_ptr_head){//ͷ������
		m_ptr_head = new_node;
	}else{
		new_node->pre->next = new_node;
	}

	if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //�������������ޣ����Զ�������������ݣ���ʱ����������������
		this->Delete(this->m_ptr_head, false);
	}
	m_n_count++;

	pthread_mutex_unlock(&m_mutex);
	return true;
}

/**
 * @brief ��pos֮�����ڵ�data
 * @param data : �����������
 * @param pos ������λ��
 * @return
 */
template<typename T>
bool ListBuffer<T>::InsertAfter(const T &data, ListNode<T> *pos){
	if(pos == nullptr)
		return false;

	//�����½ڵ�
	ListNode<T> *new_node = new ListNode<T>();
	if(new_node == nullptr)
		return false;  //����ռ�ʧ��
	new_node->data = data;
	new_node->next = nullptr;
	new_node->pre = nullptr;

	pthread_mutex_lock(&m_mutex);

	new_node->next = pos->next;
	new_node->pre = pos;
	pos->next = new_node;
	if(pos == m_ptr_tail){//β������
		m_ptr_tail = new_node;
	}else{
		new_node->next->pre = new_node;
	}

	if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //�������������ޣ����Զ�������������ݣ���ʱ����������������
		this->Delete(this->m_ptr_head, false);
	}
	m_n_count++;

	pthread_mutex_unlock(&m_mutex);

	return true;
}

/**
 * @brief ���������Ƴ�node�ڵ㣬���ǲ�ɾ��
 * @param node : ���Ƴ��Ľڵ�ָ��
 * @return
 */
template<typename T>
ListNode<T>* ListBuffer<T>::RemoveNode(ListNode<T> *node){
	if(node == nullptr)
		return nullptr;

	if(node == m_ptr_head){
		if(m_n_count == 1){//�Ƴ�Ψһ�ڵ�
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
 * @brief ��˫��������ɾ���ڵ�
 * @param T &data: ��ɾ���ڵ������
 * @return true:�ɹ�   false:ʧ��
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
		return false;   //û�ҵ�Ҫɾ���Ľڵ�
	}

	if(m_n_count == 1){//ɾ��Ψһ�ڵ�
		m_ptr_head = m_ptr_tail = nullptr;
	}
	else if(node == m_ptr_head){//ɾ��ͷ���
		m_ptr_head = node->next;
		node->next->pre = nullptr;
	}
	else if(node == m_ptr_tail){ //ɾ��β���
		m_ptr_tail = node->pre;
		node->pre->next = nullptr;
	}
	else { //ɾ���м�ڵ�
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
 * @brief ��˫��������ɾ���ڵ�
 * @param ListNode<T> * node: ��ɾ���ڵ�ָ��
 * @param mutex_test : �Ƿ���Ҫ���mutex
 * @return true:�ɹ�   false:ʧ��
 */
template<typename T>
bool ListBuffer<T>::Delete(ListNode<T> * node, bool mutex_test){

	if(node == nullptr || m_n_count == 0)
		return true;

	if(mutex_test)
		pthread_mutex_lock(&m_mutex);

	if(m_n_count == 1 && node == m_ptr_head){//ɾ��Ψһ�ڵ�
		m_ptr_head = m_ptr_tail = nullptr;
	}
	else if(node == m_ptr_head){//ɾ��ͷ���
		m_ptr_head = node->next;
		node->next->pre = nullptr;
	}
	else if(node == m_ptr_tail){ //ɾ��β���
		m_ptr_tail = node->pre;
		node->pre->next = nullptr;
	}
	else { //ɾ���м�ڵ�
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
 * @brief �������
 * @param ��
 * @return true:�ɹ�   false:ʧ��
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
 * @brief ɾ��node��ǰ�����л������ݣ�����node����
 * @param node : ָ���Ļ������ݽڵ�
 * @return true:�ɹ�   false:ʧ��
 */
template<typename T>
bool ListBuffer<T>::ClearBefore(ListNode<T> *node){
	if(m_n_count == 0)
		return true;

	pthread_mutex_lock(&m_mutex);

	ListNode<T> *node_cur = m_ptr_head;
	ListNode<T> *node_del = nullptr;
	bool bb = false;   //������־
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
 * @brief �������Ƿ��������data
 * @param data
 * @return true--������ͬ����   false--��������ͬ����
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
