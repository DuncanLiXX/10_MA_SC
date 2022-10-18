/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file CircularBuffer.h
 *@author gonghao
 *@date 2020/03/12
 *@brief ��ͷ�ļ�����ѭ��������������
 *@version
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include <assert.h>
#include <pthread.h>
#include "global_definition.h"


/**
 * @brief ѭ����������
 * ���������г��Ⱦ�ָT�ĸ������������ֽ���������
 */
template<class T>
class CircularBuffer {
public:
	CircularBuffer(int buffer_length);
	~CircularBuffer();
	int WriteData(const T* data, int data_length); //д����
	int InsertData(const T* data, int data_length); //�������ݵ�����ͷ
	int ReadData(T* data, int data_length);  //������
	int EmptyBufLen(); //��ȡ��ǰ���л���������
	int BufLen(); //��ȡ���ݳ���
	void ClearBuf();	//�������
	int ReadDataInPlace(T* data, int data_length); //��ѭ����������ȡ���ݣ����ǲ��Ƴ�����
	T* ReadDataPtr(int pos);//�����ݣ���������ָ��
	void RemoveHead();   //ɾ��ͷ������
	void RemoveData(int pos);   //�Ƴ�posλ�õ�����

private:
	T* m_buffer; //���ݻ�����ָ��
	int m_buffer_capacity;  //������������
	int m_buffer_cur_length;    //��ǰ���ݸ���
	T* m_write_pointer;      //��ǰдָ��
	T* m_read_pointer;       //��ǰ��ָ��
	pthread_mutex_t m_mutex;    //��д������
};



/**
 * @brief ���λ��������캯��
 * @param int buf_len, �������������ܴ洢T����ĸ���
 * @return ��
 */
template<class T>
CircularBuffer<T>::CircularBuffer(int buf_len) {
	m_buffer = new T[buf_len];
	if (m_buffer == NULL) {
		//TODO �����쳣
	}
	m_buffer_capacity = buf_len;
	m_buffer_cur_length = 0;
	m_write_pointer = m_buffer;
	m_read_pointer = m_buffer;
	pthread_mutex_init(&m_mutex, NULL);

}

/**
 * @brief ���λ����������������ͷſռ�
 * @param ��
 * @return ��
 */
template<class T>
CircularBuffer<T>::~CircularBuffer() {
	if (m_buffer != NULL) {
		delete[] m_buffer;
		m_buffer = NULL;
	}
	pthread_mutex_destroy(&m_mutex);
}

/**
 * @brief д�����ݵ�ѭ��������
 * @param char* data, д������ָ��
 * @param int data_length��д�����ݳ���
 * @return ����д�볤��
 */
template<class T>
int CircularBuffer<T>::WriteData(const T* data, int data_length) {
	pthread_mutex_lock(&m_mutex);

	assert(data != NULL);
	int res = 0;
	int write_length = 0;
	int tmp = 0;
	if (data_length == 0 || m_buffer == NULL) {
		goto END;
	}

	if (data_length <= m_buffer_capacity - m_buffer_cur_length) {
		write_length = data_length;
	} else {
		//������������Ѿ���������������д�룬��˴β�д���κ����ݣ����������������ݰ�������
	//	write_length = 0;
		goto END;

	}

	tmp = write_length;


	//     W|                          R|
	//------------------------------------------��ָ���ں�дָ����ǰ����������дָ���غ����
	if (m_write_pointer < m_read_pointer) {
		//memcpy(m_write_pointer, data, sizeof(T) * write_length); //��������
		//m_write_pointer += write_length; //д������ָ�����
		while(tmp > 0){
			*m_write_pointer = *data;
			m_write_pointer++;
			data++;
			tmp--;
		}
	}
	//      R|     W|
	//------------------------------------------дָ���ں󣬶�ָ����ǰ�����غϣ���дָ��֮��ʣ��ռ��㹻д����������
	//WR|
	//------------------------------------------��ʼʱ����дָ���Ϊ0
	else if (m_write_pointer - m_buffer + write_length
			<= m_buffer_capacity) {
		//memcpy(m_write_pointer, data, sizeof(T) * write_length); //��������
		//m_write_pointer += write_length; //д������ָ�����
		while(tmp > 0){
			*m_write_pointer = *data;
			m_write_pointer++;
			data++;
			tmp--;
		}
	}
	//      R|                            W|
	//------------------------------------------дָ���ں󣬶�ָ����ǰ����дָ��֮��ʣ��ռ䲻��д����������
	else {
		int split_length = m_buffer_capacity - (m_write_pointer - m_buffer);
		tmp = split_length;
//		memcpy(m_write_pointer, data, sizeof(T) * split_length);
//		memcpy(m_buffer, data + split_length,
//				sizeof(T) * (write_length - split_length));
//		m_write_pointer = m_buffer + write_length - split_length;
		while(tmp > 0){
			*m_write_pointer = *data;
			m_write_pointer++;
			data++;
			tmp--;
		}
		tmp = write_length - split_length;
		m_write_pointer = m_buffer;
		while(tmp > 0){
			*m_write_pointer = *data;
			m_write_pointer++;
			data++;
			tmp--;
		}
	}
	if (m_write_pointer >= m_buffer + m_buffer_capacity) {//дָ�볬��Χ�����
		m_write_pointer -= m_buffer_capacity;
	}
	m_buffer_cur_length += write_length; //��ǰ���ݳ��ȼ��ϴ˴�д�볤��
	res = write_length;
	END:

	pthread_mutex_unlock(&m_mutex);
	return res;
}

/**
 * @brief �������ݵ�ѭ��������ͷ��
 * @param char* data, д������ָ��
 * @param int data_length��д�����ݳ���
 * @return ����д�볤��
 */
template<class T>
int CircularBuffer<T>::InsertData(const T* data, int data_length) {
	pthread_mutex_lock(&m_mutex);
	assert(data != NULL);
	int res = 0;
	int write_length = 0;
	T *tmp_pointer = NULL;
	int tmp = 0;
	if (data_length == 0 || m_buffer == NULL) {
		goto END;
	}

	if (data_length <= m_buffer_capacity - m_buffer_cur_length) {
		write_length = data_length;
	} else {
	//	write_length = m_buffer_capacity - m_buffer_cur_length;
	//������������Ѿ���������������д�룬��˴β�д���κ����ݣ����������������ݰ�������
		goto END;
	}

	tmp = write_length;


	//     W|                          R|
	//------------------------------------------��ָ���ں�дָ����ǰ����������дָ���غ����
	if (m_write_pointer < m_read_pointer) {
		m_read_pointer -= write_length; //д������ָ�����
		tmp_pointer = m_read_pointer;
		//memcpy(m_read_pointer, data, sizeof(T) * write_length); //��������
		while(tmp > 0){
			*tmp_pointer = *data;
			tmp_pointer++;
			data++;
			tmp--;
		}
	}
	//      R|     W|
	//------------------------------------------дָ���ں󣬶�ָ����ǰ�����غϣ��Ҷ�ָ��֮ǰʣ��ռ��㹻д����������
	else if (m_read_pointer - m_buffer >= write_length) {
		m_read_pointer -= write_length;
		tmp_pointer = m_read_pointer;
		//memcpy(m_read_pointer, data, sizeof(T) * write_length); //��������
		while(tmp > 0){
			*tmp_pointer = *data;
			tmp_pointer++;
			data++;
			tmp--;
		}
	}
	//      R|                            W|
	//------------------------------------------дָ���ں󣬶�ָ����ǰ���Ҷ�ָ��֮ǰʣ��ռ䲻��д����������
	//WR|
	//------------------------------------------��ʼʱ����дָ���Ϊ0
	else {
		int split_length = write_length - (m_read_pointer - m_buffer);
		m_read_pointer = m_buffer + m_buffer_capacity - split_length;
		tmp_pointer = m_read_pointer;
		tmp = split_length;
		//memcpy(m_read_pointer, data, sizeof(T) * split_length);
		//memcpy(m_buffer, data + split_length,
		//		sizeof(T) * (write_length - split_length));
		while(tmp > 0){
			*tmp_pointer = *data;
			tmp_pointer++;
			data++;
			tmp--;
		}
		tmp = write_length - split_length;
		tmp_pointer = m_buffer;
		while(tmp > 0){
			*tmp_pointer = *data;
			tmp_pointer++;
			data++;
			tmp--;
		}
	}

	m_buffer_cur_length += write_length; //��ǰ���ݳ��ȼ��ϴ˴�д�볤��
	res = write_length;
	END:

	pthread_mutex_unlock(&m_mutex);
	return res;
}

/**
 * @brief ��ѭ����������ȡ���ݲ��Ƴ�����
 * @param char* data, ��ȡ�����ݴ�Ż�����ָ��
 * @param int data_length����Ҫ��ȡ�����ݳ���
 * @return ���ն�ȡ���ݳ���
 */
template<class T>
int CircularBuffer<T>::ReadData(T* data, int data_length) {
	pthread_mutex_lock(&m_mutex);
	assert(data != NULL);
	int res = 0;
	int read_length = 0;
	int tmp = 0;
	if (data_length == 0 || m_buffer == NULL) {
		goto END;
	}

	if (data_length < m_buffer_cur_length) {
		read_length = data_length;
	} else {
		read_length = m_buffer_cur_length;
	}

	tmp = read_length;

	//      R|                       W|
	//------------------------------------------дָ���ں󣬶�ָ����ǰ����������дָ���غ����
	if (m_write_pointer > m_read_pointer) {
	//	memcpy(data, m_read_pointer, sizeof(T) * read_length); //��������
	//	m_read_pointer += read_length; //������ָ�����
		while(tmp > 0){
			*data = *m_read_pointer;
			data++;
			m_read_pointer++;
			tmp--;
		}
	}
	//      W|          R|
	//------------------------------------------дָ����ǰ����ָ���ں��Ҷ�ָ��֮��������㹻�˴ζ�ȡ
	else if (m_buffer_capacity - (m_read_pointer - m_buffer)
			>= read_length) {
		//memcpy(data, m_read_pointer, sizeof(T) * read_length); //��������
		//m_read_pointer += read_length; //������ָ�����
		while(tmp > 0){
			*data = *m_read_pointer;
			data++;
			m_read_pointer++;
			tmp--;
		}
		if (m_read_pointer >= m_buffer + m_buffer_capacity) {
			m_read_pointer -= m_buffer_capacity;
		}
	}
	//      W|                             R|
	//------------------------------------------дָ����ǰ����ָ���ں��Ҷ�ָ��֮������ݲ����˴ζ�ȡ
	else {
		int split_length = m_buffer_capacity - (m_read_pointer - m_buffer);
//		memcpy(data, m_read_pointer, sizeof(T) * split_length);
//		memcpy(data + split_length, m_buffer,
//				sizeof(T) * (read_length - split_length));
//		m_read_pointer = m_buffer + read_length - split_length;
		tmp = split_length;
		while(tmp > 0){
			*data = *m_read_pointer;
			data++;
			m_read_pointer++;
			tmp--;
		}
		m_read_pointer = m_buffer;
		tmp = read_length - split_length;
		while(tmp > 0){
			*data = *m_read_pointer;
			data++;
			m_read_pointer++;
			tmp--;
		}

	}
	m_buffer_cur_length -= read_length; //��ǰ���ݳ��ȼ�ȥ�˴ζ�ȡ����
	res = read_length;

	END:
//	g_trace_info->PrintTrace(TRACE_INFO, CIRCULAR_BUFFER,
//			"<--CircularBuffer::ReadData");
	pthread_mutex_unlock(&m_mutex);
	return res;
}

/**
 * @brief ��ѭ����������ȡ���ݣ����ǲ��Ƴ�����
 * @param char* data, ��ȡ�����ݴ�Ż�����ָ��
 * @param int data_length����Ҫ��ȡ�����ݳ���
 * @return ���ն�ȡ���ݳ���
 */
template<class T>
int CircularBuffer<T>::ReadDataInPlace(T* data, int data_length) {
	pthread_mutex_lock(&m_mutex);

	assert(data != NULL);
	int res = 0;
	int read_length = 0;
	T *tmp_pointer = m_read_pointer;
	int tmp = 0;
	if (data_length == 0 || m_buffer == NULL) {
		goto END;
	}

	if (data_length < m_buffer_cur_length) {
		read_length = data_length;
	} else {
		read_length = m_buffer_cur_length;
	}

	tmp = read_length;

	//      R|                       W|
	//------------------------------------------дָ���ں󣬶�ָ����ǰ����������дָ���غ����
	if (m_write_pointer > m_read_pointer) {
		//memcpy(data, m_read_pointer, sizeof(T) * read_length); //��������
		//m_read_pointer += read_length; //������ָ�����
		while(tmp > 0){
			*data = *tmp_pointer;
			data++;
			tmp_pointer++;
			tmp--;
		}

	}
	//      W|          R|
	//------------------------------------------дָ����ǰ����ָ���ں��Ҷ�ָ��֮��������㹻�˴ζ�ȡ
	else if (m_buffer_capacity - (m_read_pointer - m_buffer)
			>= read_length) {
		//memcpy(data, m_read_pointer, sizeof(T) * read_length); //��������
		//m_read_pointer += read_length; //������ָ�����
		//if (m_read_pointer >= m_buffer + m_buffer_capacity) {
		//	m_read_pointer -= m_buffer_capacity;
		//}
		while(tmp > 0){
			*data = *tmp_pointer;
			data++;
			tmp_pointer++;
			tmp--;
		}
	}
	//      W|                             R|
	//------------------------------------------дָ����ǰ����ָ���ں��Ҷ�ָ��֮������ݲ����˴ζ�ȡ
	else {
		int split_length = m_buffer_capacity - (m_read_pointer - m_buffer);
	//	memcpy(data, m_read_pointer, sizeof(T) * split_length);
	//	memcpy(data + split_length, m_buffer,
	//			sizeof(T) * (read_length - split_length));
		//m_read_pointer = m_buffer + read_length - split_length;
		tmp = split_length;
		while(tmp > 0){
			*data = *tmp_pointer;
			data++;
			tmp_pointer++;
			tmp--;
		}
		tmp_pointer = m_buffer;
		tmp = read_length - split_length;
		while(tmp > 0){
			*data = *tmp_pointer;
			data++;
			tmp_pointer++;
			tmp--;
		}
	}
	//m_buffer_cur_length -= read_length; //��ǰ���ݳ��ȼ�ȥ�˴ζ�ȡ����
	res = read_length;

	END:

	pthread_mutex_unlock(&m_mutex);
	return res;
}

/**
 * @brief ��ȡѭ�����������г���
 * @param ��
 * @return ���г���
 */
template<class T>
int CircularBuffer<T>::EmptyBufLen() {
	pthread_mutex_lock(&m_mutex);

	int empty_buf_len = m_buffer_capacity - m_buffer_cur_length;

	pthread_mutex_unlock(&m_mutex);
	return empty_buf_len;
}

/**
 * @brief ��ȡѭ�����������ݳ���
 * @param ��
 * @return ���ݳ���
 */
template<class T>
int CircularBuffer<T>::BufLen() {
	return m_buffer_cur_length;
}

/**
 * @brief ��ȡѭ�����������ݳ���
 * @param ��
 * @return ���ݳ���
 */
template<class T>
void CircularBuffer<T>::ClearBuf() {
	pthread_mutex_lock(&m_mutex);

	m_buffer_cur_length = 0;
	m_write_pointer = m_buffer;
	m_read_pointer = m_buffer;

	pthread_mutex_unlock(&m_mutex);
}

/**
 * @brief ��ѭ����������ȡ����ָ��
 * @param int pos����Ҫ��ȡ���ݵ�λ��
 * @return ��ȡ������ָ�룬û���������򷵻�null
 */
template<class T>
T* CircularBuffer<T>::ReadDataPtr(int pos) {
	pthread_mutex_lock(&m_mutex);

	T* pointer = NULL;

	if (pos >= 0 && pos < m_buffer_cur_length) {
		pointer = m_read_pointer + pos;
		if (pointer >= m_buffer + m_buffer_capacity) {
			pointer -= m_buffer_capacity;
		}
	}

	pthread_mutex_unlock(&m_mutex);

	return pointer;
}

/**
 * @brief ��ѭ��������ɾ��ͷ�����ݣ���ɾ����ǰ��ȡָ��λ�õ�����
 * @param ��
 * @return ��
 */
template<class T>
void CircularBuffer<T>::RemoveHead() {
	pthread_mutex_lock(&m_mutex);

	m_read_pointer++;
	if(m_read_pointer >= (m_buffer+m_buffer_capacity))
		m_read_pointer = m_buffer;

	m_buffer_cur_length--;


	pthread_mutex_unlock(&m_mutex);
}

/**
 * @brief �Ƴ�posλ�õ�����
 * @param pos : ��Ҫ�Ƴ����ݵ�λ��
 */
template<class T>
void CircularBuffer<T>::RemoveData(int pos){
    if(pos == 0){  //�����ͷ������ֱ���Ƴ�
        this->RemoveHead();
        return;
    }

    pthread_mutex_lock(&m_mutex);

    T* pointer = NULL, *ppm = NULL;
    int count = this->BufLen()-pos-1;  //��Ų��������

    if (pos >= 0 && pos < m_buffer_cur_length) {
        pointer = m_read_pointer + pos;
        if (pointer >= (m_buffer + m_buffer_capacity)) {
            pointer -= m_buffer_capacity;
        }

        if (count == 0)
        {// ɾ�����һ��Ԫ��
            this->m_write_pointer--;
            m_buffer_cur_length--;
        }
        else
        {
            while(count > 0){
                ppm = pointer+1;
                if(ppm >= m_buffer + m_buffer_capacity)
                    ppm -= m_buffer_capacity;

                *pointer = *ppm;
                pointer = ppm;
                count--;
            }

            this->m_write_pointer = ppm;
            m_buffer_cur_length--;
        }
    }

    pthread_mutex_unlock(&m_mutex);
}



#endif /* CIRCULARBUFFER_H_ */
