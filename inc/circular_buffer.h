/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file CircularBuffer.h
 *@author gonghao
 *@date 2020/03/12
 *@brief 本头文件包含循环缓冲区类声明
 *@version
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include <assert.h>
#include <pthread.h>
#include "global_definition.h"


/**
 * @brief 循环缓冲区类
 * 此类中所有长度均指T的个数，并不是字节数！！！
 */
template<class T>
class CircularBuffer {
public:
	CircularBuffer(int buffer_length);
	~CircularBuffer();
	int WriteData(const T* data, int data_length); //写数据
	int InsertData(const T* data, int data_length); //插入数据到队列头
	int ReadData(T* data, int data_length);  //读数据
	int EmptyBufLen(); //获取当前空闲缓冲区长度
	int BufLen(); //获取数据长度
	void ClearBuf();	//清空数据
	int ReadDataInPlace(T* data, int data_length); //从循环缓冲区读取数据，但是不移除数据
	T* ReadDataPtr(int pos);//读数据，返回数据指针
	void RemoveHead();   //删除头部数据
	void RemoveData(int pos);   //移除pos位置的数据

private:
	T* m_buffer; //数据缓冲区指针
	int m_buffer_capacity;  //缓冲区总容量
	int m_buffer_cur_length;    //当前数据个数
	T* m_write_pointer;      //当前写指针
	T* m_read_pointer;       //当前读指针
	pthread_mutex_t m_mutex;    //读写互斥量
};



/**
 * @brief 环形缓冲区构造函数
 * @param int buf_len, 缓冲区容量，能存储T对象的个数
 * @return 无
 */
template<class T>
CircularBuffer<T>::CircularBuffer(int buf_len) {
	m_buffer = new T[buf_len];
	if (m_buffer == NULL) {
		//TODO 处理异常
	}
	m_buffer_capacity = buf_len;
	m_buffer_cur_length = 0;
	m_write_pointer = m_buffer;
	m_read_pointer = m_buffer;
	pthread_mutex_init(&m_mutex, NULL);

}

/**
 * @brief 环形缓冲区析构函数，释放空间
 * @param 无
 * @return 无
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
 * @brief 写入数据到循环缓冲区
 * @param char* data, 写入数据指针
 * @param int data_length，写入数据长度
 * @return 最终写入长度
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
		//如果缓冲区中已经不够将所有数据写入，则此次不写入任何数据，否则会造成命令数据包不完整
	//	write_length = 0;
		goto END;

	}

	tmp = write_length;


	//     W|                          R|
	//------------------------------------------读指针在后，写指针在前，不包含读写指针重合情况
	if (m_write_pointer < m_read_pointer) {
		//memcpy(m_write_pointer, data, sizeof(T) * write_length); //复制数据
		//m_write_pointer += write_length; //写入数据指针后移
		while(tmp > 0){
			*m_write_pointer = *data;
			m_write_pointer++;
			data++;
			tmp--;
		}
	}
	//      R|     W|
	//------------------------------------------写指针在后，读指针在前，或重合，且写指针之后剩余空间足够写入所有数据
	//WR|
	//------------------------------------------初始时，读写指针均为0
	else if (m_write_pointer - m_buffer + write_length
			<= m_buffer_capacity) {
		//memcpy(m_write_pointer, data, sizeof(T) * write_length); //复制数据
		//m_write_pointer += write_length; //写入数据指针后移
		while(tmp > 0){
			*m_write_pointer = *data;
			m_write_pointer++;
			data++;
			tmp--;
		}
	}
	//      R|                            W|
	//------------------------------------------写指针在后，读指针在前，且写指针之后剩余空间不够写入所有数据
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
	if (m_write_pointer >= m_buffer + m_buffer_capacity) {//写指针超范围后回退
		m_write_pointer -= m_buffer_capacity;
	}
	m_buffer_cur_length += write_length; //当前数据长度加上此次写入长度
	res = write_length;
	END:

	pthread_mutex_unlock(&m_mutex);
	return res;
}

/**
 * @brief 插入数据到循环缓冲区头部
 * @param char* data, 写入数据指针
 * @param int data_length，写入数据长度
 * @return 最终写入长度
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
	//如果缓冲区中已经不够将所有数据写入，则此次不写入任何数据，否则会造成命令数据包不完整
		goto END;
	}

	tmp = write_length;


	//     W|                          R|
	//------------------------------------------读指针在后，写指针在前，不包含读写指针重合情况
	if (m_write_pointer < m_read_pointer) {
		m_read_pointer -= write_length; //写入数据指针后移
		tmp_pointer = m_read_pointer;
		//memcpy(m_read_pointer, data, sizeof(T) * write_length); //复制数据
		while(tmp > 0){
			*tmp_pointer = *data;
			tmp_pointer++;
			data++;
			tmp--;
		}
	}
	//      R|     W|
	//------------------------------------------写指针在后，读指针在前，或重合，且读指针之前剩余空间足够写入所有数据
	else if (m_read_pointer - m_buffer >= write_length) {
		m_read_pointer -= write_length;
		tmp_pointer = m_read_pointer;
		//memcpy(m_read_pointer, data, sizeof(T) * write_length); //复制数据
		while(tmp > 0){
			*tmp_pointer = *data;
			tmp_pointer++;
			data++;
			tmp--;
		}
	}
	//      R|                            W|
	//------------------------------------------写指针在后，读指针在前，且读指针之前剩余空间不够写入所有数据
	//WR|
	//------------------------------------------初始时，读写指针均为0
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

	m_buffer_cur_length += write_length; //当前数据长度加上此次写入长度
	res = write_length;
	END:

	pthread_mutex_unlock(&m_mutex);
	return res;
}

/**
 * @brief 从循环缓冲区读取数据并移除数据
 * @param char* data, 读取的数据存放缓冲区指针
 * @param int data_length，需要读取的数据长度
 * @return 最终读取数据长度
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
	//------------------------------------------写指针在后，读指针在前，不包含读写指针重合情况
	if (m_write_pointer > m_read_pointer) {
	//	memcpy(data, m_read_pointer, sizeof(T) * read_length); //复制数据
	//	m_read_pointer += read_length; //读数据指针后移
		while(tmp > 0){
			*data = *m_read_pointer;
			data++;
			m_read_pointer++;
			tmp--;
		}
	}
	//      W|          R|
	//------------------------------------------写指针在前，读指针在后，且读指针之后的数据足够此次读取
	else if (m_buffer_capacity - (m_read_pointer - m_buffer)
			>= read_length) {
		//memcpy(data, m_read_pointer, sizeof(T) * read_length); //复制数据
		//m_read_pointer += read_length; //读数据指针后移
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
	//------------------------------------------写指针在前，读指针在后，且读指针之后的数据不够此次读取
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
	m_buffer_cur_length -= read_length; //当前数据长度减去此次读取长度
	res = read_length;

	END:
//	g_trace_info->PrintTrace(TRACE_INFO, CIRCULAR_BUFFER,
//			"<--CircularBuffer::ReadData");
	pthread_mutex_unlock(&m_mutex);
	return res;
}

/**
 * @brief 从循环缓冲区读取数据，但是不移除数据
 * @param char* data, 读取的数据存放缓冲区指针
 * @param int data_length，需要读取的数据长度
 * @return 最终读取数据长度
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
	//------------------------------------------写指针在后，读指针在前，不包含读写指针重合情况
	if (m_write_pointer > m_read_pointer) {
		//memcpy(data, m_read_pointer, sizeof(T) * read_length); //复制数据
		//m_read_pointer += read_length; //读数据指针后移
		while(tmp > 0){
			*data = *tmp_pointer;
			data++;
			tmp_pointer++;
			tmp--;
		}

	}
	//      W|          R|
	//------------------------------------------写指针在前，读指针在后，且读指针之后的数据足够此次读取
	else if (m_buffer_capacity - (m_read_pointer - m_buffer)
			>= read_length) {
		//memcpy(data, m_read_pointer, sizeof(T) * read_length); //复制数据
		//m_read_pointer += read_length; //读数据指针后移
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
	//------------------------------------------写指针在前，读指针在后，且读指针之后的数据不够此次读取
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
	//m_buffer_cur_length -= read_length; //当前数据长度减去此次读取长度
	res = read_length;

	END:

	pthread_mutex_unlock(&m_mutex);
	return res;
}

/**
 * @brief 获取循环缓冲区空闲长度
 * @param 无
 * @return 空闲长度
 */
template<class T>
int CircularBuffer<T>::EmptyBufLen() {
	pthread_mutex_lock(&m_mutex);

	int empty_buf_len = m_buffer_capacity - m_buffer_cur_length;

	pthread_mutex_unlock(&m_mutex);
	return empty_buf_len;
}

/**
 * @brief 获取循环缓冲区数据长度
 * @param 无
 * @return 数据长度
 */
template<class T>
int CircularBuffer<T>::BufLen() {
	return m_buffer_cur_length;
}

/**
 * @brief 获取循环缓冲区数据长度
 * @param 无
 * @return 数据长度
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
 * @brief 从循环缓冲区读取数据指针
 * @param int pos，需要读取数据的位置
 * @return 读取的数据指针，没读到数据则返回null
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
 * @brief 从循环缓冲区删除头部数据，即删除当前读取指针位置的数据
 * @param 无
 * @return 无
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
 * @brief 移除pos位置的数据
 * @param pos : 需要移除数据的位置
 */
template<class T>
void CircularBuffer<T>::RemoveData(int pos){
    if(pos == 0){  //如果是头部，则直接移除
        this->RemoveHead();
        return;
    }

    pthread_mutex_lock(&m_mutex);

    T* pointer = NULL, *ppm = NULL;
    int count = this->BufLen()-pos-1;  //待挪动数据量

    if (pos >= 0 && pos < m_buffer_cur_length) {
        pointer = m_read_pointer + pos;
        if (pointer >= (m_buffer + m_buffer_capacity)) {
            pointer -= m_buffer_capacity;
        }

        if (count == 0)
        {// 删除最后一个元素
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
