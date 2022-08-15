/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file DataStack.h
 *@author gonghao
 *@date 2020/04/01
 *@brief ��Դ�ļ�Ϊ����ջģ���������
 *@version
 */

#ifndef DATASTACK_H_
#define DATASTACK_H_

//��ջ����Ԫ�ṹ,T�����Ƕ���Ҳ�����Ƕ���ָ��
template<class T>
struct StackRec {
    T rec;   //���ݶ���ָ��
    StackRec* down;    //����һ������Ԫ
    StackRec* up;    //��һ������Ԫ
    StackRec(T &data):down(nullptr),up(nullptr) {rec = data;}
};

/**
 * @brief ���ݴ洢��ջ
 */
template<class T>
class DataStack {
public://�����ӿں���
	DataStack(int max = 0);
	virtual ~DataStack();

	bool pop(T &data);       //����ջ������
	void pop();             //����ջ�����ݲ�����
	void pop2();			//����ջ��Ԫ�ص���������ͷſռ�
	bool cur(T &data);     //��ȡջ�����ݣ����ǲ�����
	bool bottom(T &data);  //��ȡջ�����ݣ���������
	bool push(T &data);  //��ջ����
	void empty();    //���ջ������
	int size();      //���ݸ���
	void set_max_size(int max);   // �����������
	int max_size() const{return m_n_max_count;}      //��ȡ��ջ�������

	StackRec<T> *top()const {return m_p_top;}   //ջ������Ԫָ��
	StackRec<T> *bottom()const {return m_p_bottom;}   //ջ������Ԫָ��

	DataStack<T> &operator=(const DataStack<T> &st);   //���ظ�ֵ�����

private://˽�нӿں���
//	virtual void freeItem( T *data );   //�ͷ�����Ԫ�ڴ�ռ�

	//����ָ��Ͷ������ִ���
	void DeleteData(std::true_type, T data){
		printf("delete data1\n");
		delete data;
		printf("delete data2\n");
	}
	void DeleteData(std::false_type, T data){}
private://˽�г�Ա����
	StackRec<T> *m_p_top;     //ջ��ָ��
	StackRec<T> *m_p_bottom;  //ջ��ָ��
	int  m_n_count;       //ջ����������
	int m_n_max_count;     //ջ��������Ϊ0���ʾ�����ƣ�������ֵʱ�Զ��������������
};

/**
 * @brief ���ݶ�ջ�๹�캯��
 * @param bool bPointer:��־����T�Ƿ�Ϊָ����
 */
template<class T>
DataStack<T>::DataStack(int max): m_p_top(nullptr),m_p_bottom(nullptr),m_n_count(0),m_n_max_count(max) {
	// TODO Auto-generated constructor stub
}

/**
 * @brief ���ݶ�ջ����������
 */
template<class T>
DataStack<T>::~DataStack() {
	// TODO Auto-generated destructor stub
	empty();
}


/**
 * @brief ��ջ�����������������ݷ����ջ����
 * @param T &data: ��Ҫ�����ջ������
 * @return true--�ɹ��� false--ʧ��
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


        if(m_n_max_count > 0 && m_n_count == m_n_max_count){  //ջ�����������ޣ����Զ��������������
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
 * @brief ����ջ������
 * @param data : ջ������
 * @return true--�����ɹ�   false--����ʧ�ܣ������ݿ�ȡ
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
 * @brief ����ջ�����ݲ�����
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
 * @brief ����ջ��Ԫ�ص���������ͷſռ�
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
 * @brief ��ȡջ�����ݣ����ǲ�����
 * @param data : ջ�����ݵ�����
 * @return false��������  true--�ɹ�
 */
template<class T>
bool DataStack<T>::cur(T &data)
{
	if(m_n_count <= 0) return false;
	data = m_p_top->rec;
    return true;
}

/**
 * @brief ��ȡջ�����ݣ���������
 * @param data : ջ�����ݵ�����
 * @return false��������  true--�ɹ�
 */
template<class T>
bool DataStack<T>::bottom(T &data){
	if(m_n_count <= 0) return false;
	data = m_p_bottom->rec;
    return true;
}

/**
 * @brief ��ǰ��ջ���ݸ���
 */
template<class T>
int DataStack<T>::size(){
	return m_n_count;
}


/**
 * @brief ���ջ������
 */
template<class T>
void DataStack<T>::empty()
{
    while( m_n_count ) {
    	pop();

    }
}



/**
 * @brief ���ظ�ֵ�����
 * @param list
 * @return
 */
template<typename T>
DataStack<T> &DataStack<T>::operator=(const DataStack<T> &st){
	if(&st == this)
		return *this;
	empty();

	this->set_max_size(st.max_size());   //�����������

	StackRec<T> *node = st.bottom();
	while(node != nullptr){
		this->push(node->rec);

		node = node->up;
	}

	return *this;
}


/**
 * @brief ���õ�ǰ��ջ�������
 */
template<class T>
void DataStack<T>::set_max_size(int max){
	m_n_max_count = max;

	//ɾ����������������
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
