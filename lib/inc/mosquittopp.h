/*
Copyright (c) 2010-2019 Roger Light <roger@atchoo.org>

All rights reserved. This program and the accompanying materials
are made available under the terms of the Eclipse Public License 2.0
and Eclipse Distribution License v1.0 which accompany this distribution.

The Eclipse Public License is available at
   https://www.eclipse.org/legal/epl-2.0/
and the Eclipse Distribution License is available at
  http://www.eclipse.org/org/documents/edl-v10.php.

Contributors:
   Roger Light - initial implementation and documentation.
*/

#ifndef MOSQUITTOPP_H
#define MOSQUITTOPP_H

#if defined(_WIN32) && !defined(LIBMOSQUITTO_STATIC)
#	ifdef mosquittopp_EXPORTS
#		define mosqpp_EXPORT  __declspec(dllexport)
#	else
#		define mosqpp_EXPORT  __declspec(dllimport)
#	endif
#else
#	define mosqpp_EXPORT
#endif

#include <cstdlib>
#include <mosquitto.h>
#include <time.h>
#include <pthread.h>

namespace mosqpp {

mosqpp_EXPORT int lib_version(int *major, int *minor, int *revision);
mosqpp_EXPORT int lib_init();
mosqpp_EXPORT int lib_cleanup();
mosqpp_EXPORT int validate_utf8(const char *str, int len);

/*
 * Class: mosquittopp
 *
 */
class mosqpp_EXPORT mosquittopp {
    private:
        struct mosquitto *m_mosq;
        bool m_isConnected = false;
        pthread_mutex_t m_mutex;
    public:
        mosquittopp(const char *id=NULL, bool clean_session=true);
        virtual ~mosquittopp();

        void setConnected(bool bConnected);
        bool isConnected();

        int connect(const char *host, int port=1883, int keepalive=60);
        int disconnect();

        int will_set(const char *topic, int payloadlen=0, const void *payload=NULL, int qos=0, bool retain=false);
        int will_clear();

        int publish(int *mid, const char *topic, int payloadlen=0, const void *payload=NULL, int qos=0, bool retain=false);
        int subscribe(int *mid, const char *sub, int qos=0);
        int unsubscribe(int *mid, const char *sub);

        int loop_forever(int timeout=-1, int max_packets=1);
        int loop_start();
        int loop_stop(bool force=false);

        virtual void on_connect(int /*rc*/) {return;}
        virtual void on_disconnect(int /*rc*/) {return;}
        virtual void on_publish(int /*mid*/) {return;}
        virtual void on_message(const struct mosquitto_message * /*message*/) {return;}
        virtual void on_subscribe(int /*mid*/, int /*qos_count*/, const int * /*granted_qos*/) {return;}
        virtual void on_unsubscribe(int /*mid*/) {return;}
};

}
#endif
