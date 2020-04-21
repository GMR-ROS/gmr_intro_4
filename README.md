# Introdução de ROS e robôs móveis terrestres

Após incrementarmos RobotClass com o [publicador de odometria](https://github.com/akihirohh/gmr_intro_3), vamos explorar um pouco de serviços. 

## ROS Services

Os serviços ROS são uma alternativa ao modelo publicador-subscritor, em que a comunicação é contínua após ser estabelecida, mas é de via única: o publicador manda mensagem ao subscritor, mas o publicador em si não consegue saber nada do seu subscritor. Já os serviços são eventos pontuais, em que um cliente faz uma requisição ao servidor. Análogo ao mundo real, é possível que o servidor retorne uma resposta à requisição feita. 

Assim como os tópicos utilizam mensagens ROS para carregar suas informações, os serviços possuem suas formas estruturadas de informação. Por exemplo, informações sobre o serviço roscpp_tutorials/TwoInts podem ser obtidas com: 
```console
user@pc:~/$ rossrv info roscpp_tutorials/TwoInts
int64 a
int64 b
---
int64 sum
```

Observe que a descrição de um serviço possui duas seções, delimitadas pelo marcador `---`. A primeira seção possui os dados referentes ao cliente. Já a segunda contém a resposta envia pelo servidor após executar o serviço. Neste caso, o cliente envia dois inteiros, a e b, e o servidor retorna a soma deles, sum. Agora observe o serviço std_srvs/Trigger:
```console
user@pc:~/$ rossrv info std_srvs/Trigger
---
bool success
string message
```
Como o próprio nome indica, trata-se apenas de um trigger. O cliente só faz a solicitação do serviço sem enviar nenhum dado. Por outro lado, o servidor responde com um booleano indicando se o serviço foi bem-sucedido e com uma string. 

Da mesma forma que as mensagens ROS, podemos encontrar algumas que já vêm instaladas. Mas são poucas e muito mais difíceis de generalizar. Caso necessário, serviços personalizados podem ser criados no seu pacote. 

## Nosso caso
Ao rodar `roslaunch gmr_intro_poo robot.launch` ou simplesmente `rosrun gmr_intro gmr_intro_node` e, numa sessão adicional de terminal, executar:
```console
user@pc:~$  rosservice list
/gmr_intro_node/get_loggers
/gmr_intro_node/set_logger_level
/gmr_intro_poo_node/get_loggers
/gmr_intro_poo_node/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
/toggle_robot
```
Observamos que cada nó ativo possui ao menos dois serviços associados: get_loggers e set_logger_level. Eles estão relacionados às mensagens de saída ao console/[logging](), ou seja, quando utilizamos ROS_INFO ou ROS_INFO_STREAM, por exemplo. Porém, temos um outro serviço disponível: `/toggle_robot`. Para obter informações sobre ele:
```console
user@pc:~$ rosservice info /toggle_robot 
Node: /gmr_intro_node
URI: rosrpc://pc:42811
Type: std_srvs/Trigger
Args: 
```
Obtemos qual o nó que é servidor, a combinação hostname:port (pc:42811) pela qual a comunicação será estabelecida, o tipo de serviço e os argumentos necessários para fazer uma requisição do serviço. Como esperado (vimos acima com rossrv info std_srvs/Trigger), trata-se de um serviço que não precisa de argumentos para ser solicitado. Podemos, por exemplo, fazer uma requisição do serviço /toggle_robot através da linha de comando:
```console
user@pc:~$ rosservice call /toggle_robot 
success: True
message: "Someone toggled me..."
```
Observe que recebemos a resposta do servidor e se olharmos a saída de nosso gmr_intro_poo_node, as velocidades estão zeradas. De fato, `rostopic echo /left_rpm` e `rostopic echo /right_rpm` confirmam isso. Se fizermos a requisição do serviço novamente, o robô volta a se movimentar.

## Requisição de serviços dentro de um nó ROS em C++

Veremos agora como fazer a requisição do serviço por código. Vamos incrementar o nosso RobotClass para que ele chame /toggle_robot a cada n segundos, em que n é um parâmetro ROS. Primeiro, editaremos o cabeçalho em gmr_intro_poo/include/gmr_intro_poo/gmr_intro_poo.hpp, adicionando a biblioteca relacionada ao serviço:

```cpp
#include <std_srvs/Trigger.h>
```
Adicionaremos o protótipo da função que fará a requisição observando o intervalo de tempo:
```cpp
        void                checkToggleRobot();
```        
Adicionaremos um cliente de serviço como membro privado:
```cpp
        ros::ServiceClient  _client_toggle_robot;
```
Como desejamos que a requisição do serviço ocorra em tempos regulares e definidos por um parâmetro, adicionemos um campo na struct Params:
```cpp

        struct Params
        {
            double          time_between_toggle;
            double          axle_track;
            double          gear_ratio;
            double          wheel_radius;
            std::string     topic_name_left_rpm;
            std::string     topic_name_right_rpm;
        }_params;
```
E também um membro privado para manter o tempo da última requisição:
```cpp
        double _prev_timestamp_toggle;
```
Agora editaremos a implementação da classe em gmr_intro_poo/src/gmr_intro_poo.cpp. No construtor da classe RobotClass::RobotClass(ros::NodeHandle* nh), precisamos inicializar o cliente, obter o parâmetro e atribuir o valor inicial ao _prev_timestamp_toggle.
```cpp

    _nh->param("time_between_toggles", _param.time_between_toggles, -1.0);

    _client_toggle_robot = _nh->serviceClient<std_srvs::Trigger>("/toggle_service");

    _prev_timestamp_toggle = _prev_timestamp;
```

Vamos criar o método que 

```cpp
void RobotClass::checkToggleRobot()
{
    double cur_timestamp = ros::Time::now().toSec();
    std_srvs::Trigger srv;
    if(_params.time_between_toggles > 0  && cur_timestamp - _prev_timestamp_toggle > _params.time_between_toggles)
    {
        _prev_timestamp_toggle = cur_timestamp;
        _client_toggle_robot.call(srv);
        ROS_WARN_STREAM("message: " << srv.response.message);
    }
}
```



e em 
```cpp
    while(ros::ok())
    {
        robot.checkToggleRobot();
        robot.calculateOdom();
        loop_rate.sleep();
        ros::spinOnce();
    }
```