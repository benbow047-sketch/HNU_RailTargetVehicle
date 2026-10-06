#include "drv_msg.h"

static uint8_t idx = 0;  // 记录当前注册的话题数量
static topic_t *topic_obj[MAX_TOPIC_COUNT] = {NULL};  // 保存话题实例指针

/**
 * @brief 检查话题实例是否重名
 * @param name 话题名称
 * @return 0:不存在重名; 非0:存在重名(返回索引+1)
 */
static uint8_t check_topic_name(char *name) {
    for (int i = 0; i < MAX_TOPIC_COUNT; i++) {
        if (topic_obj[i] != NULL) {
            if (strcmp(topic_obj[i]->name, name) == 0) {
                return i + 1;  // 存在重名，返回索引+1
            }
        }
    }
    return 0;  // 不存在重名
}

/**
 * @brief 注册成为消息发布者
 * @param name 话题名称
 * @param len 消息类型长度(sizeof获取)
 * @return 发布者实例指针，NULL表示失败
 */
publisher_t *pub_register(char *name, uint8_t len) {
    if (name == NULL || len == 0 || idx >= MAX_TOPIC_COUNT) {
        printf("pub_register: invalid parameter or topic full\r\n");
        return NULL;
    }

    uint8_t check_num = check_topic_name(name);
    publisher_t *pub = (publisher_t *)pvPortMalloc(sizeof(publisher_t));
    if (pub == NULL) {
        printf("pub_register: malloc publisher failed\r\n");
        return NULL;
    }
    memset(pub, 0, sizeof(publisher_t));

    if (!check_num) {  // 不存在重名话题，创建新话题
        topic_obj[idx] = (topic_t *)pvPortMalloc(sizeof(topic_t));
        if (topic_obj[idx] == NULL) {
            printf("pub_register: malloc topic failed\r\n");
            vPortFree(pub);
            return NULL;
        }
        memset(topic_obj[idx], 0, sizeof(topic_t));

        // 分配消息存储内存
        topic_obj[idx]->msg = pvPortMalloc(len);
        if (topic_obj[idx]->msg == NULL) {
            printf("pub_register: malloc msg buffer failed\r\n");
            vPortFree(topic_obj[idx]);
            vPortFree(pub);
            return NULL;
        }

        // 创建互斥信号量
        topic_obj[idx]->sem = xSemaphoreCreateMutex();
        if (topic_obj[idx]->sem == NULL) {
            printf("pub_register: create semaphore failed\r\n");
            vPortFree(topic_obj[idx]->msg);
            vPortFree(topic_obj[idx]);
            vPortFree(pub);
            return NULL;
        }

        // 初始化话题名称
        strncpy(topic_obj[idx]->name, name, MSG_NAME_MAX - 1);
        topic_obj[idx]->name[MSG_NAME_MAX - 1] = '\0';

        // 绑定发布者与话题
        pub->tp = topic_obj[idx];
        pub->topic_name = topic_obj[idx]->name;
        pub->len = len;
        idx++;
    } else {  // 存在重名话题，直接绑定
        pub->tp = topic_obj[check_num - 1];
        pub->topic_name = topic_obj[check_num - 1]->name;
        pub->len = len;
    }

    return pub;
}

/**
 * @brief 订阅话题消息
 * @param name 话题名称
 * @param len 消息长度(sizeof获取)
 * @return 订阅者实例指针，NULL表示失败
 */
subscriber_t *sub_register(char *name, uint8_t len) {
    // 复用发布者注册逻辑，共享话题实例
    publisher_t *pub = pub_register(name, len);
    if (pub == NULL) {
        return NULL;
    }

    // 转换为订阅者实例
    subscriber_t *sub = (subscriber_t *)pvPortMalloc(sizeof(subscriber_t));
    if (sub == NULL) {
        printf("sub_register: malloc subscriber failed\r\n");
        return NULL;
    }
    memset(sub, 0, sizeof(subscriber_t));

    sub->tp = pub->tp;
    sub->topic_name = pub->topic_name;
    sub->len = pub->len;

    // 释放临时的发布者实例(仅复用了注册逻辑)
    vPortFree(pub);
    return sub;
}

/**
 * @brief 发布消息
 * @param pub 发布者实例
 * @param data 待发布的消息数据
 * @return 1:成功 0:失败
 */
uint8_t pub_push_msg(publisher_t *pub, void *data) {
    if (pub == NULL || data == NULL || pub->tp == NULL) {
        printf("pub_push_msg: invalid parameter\r\n");
        return 0;
    }

    // 获取互斥锁，超时时间0(不等待)
    if (xSemaphoreTake(pub->tp->sem, 0) != pdTRUE) {
        printf("pub_push_msg: take semaphore failed\r\n");
        return 0;
    }

    // 复制消息数据
    memcpy(pub->tp->msg, data, pub->len);

    // 释放互斥锁
    xSemaphoreGive(pub->tp->sem);
    return 1;
}

/**
 * @brief 获取订阅的消息
 * @param sub 订阅者实例
 * @param data 接收消息的缓冲区
 * @return 1:成功 0:失败
 */
uint8_t sub_get_msg(subscriber_t *sub, void *data) {
    if (sub == NULL || data == NULL || sub->tp == NULL) {
        printf("sub_get_msg: invalid parameter\r\n");
        return 0;
    }

    // 获取互斥锁，超时时间0(不等待)
    if (xSemaphoreTake(sub->tp->sem, 0) != pdTRUE) {
        printf("sub_get_msg: take semaphore failed\r\n");
        return 0;
    }

    // 复制消息数据
    memcpy(data, sub->tp->msg, sub->len);

    // 释放互斥锁
    xSemaphoreGive(sub->tp->sem);
    return 1;
}
