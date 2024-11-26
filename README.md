# Instruções de Configuração do Projeto ROS

Este README fornece instruções passo a passo para clonar, compilar e executar seus projetos ROS 2.

## Pré-requisitos

Certifique-se de ter os seguintes itens instalados:
- [ROS 2](https://docs.ros.org/en/ros2)
- [Pygame](https://www.pygame.org/)

Instale o Pygame via pip, caso ainda não esteja instalado:
```bash
pip install pygame
```

---

## Primeiro passo

1. Clone o repositório:
   ```bash
   git clone https://github.com/rmnicola/Culling_Games cg
   ```

2. Navegue até o diretório do projeto:
   ```bash
   cd cg
   ```

3. Compile o projeto:
   ```bash
   colcon build
   ```

4. Fonteie o arquivo de configuração:
   - Se estiver usando `zsh`:
     ```bash
     source install/local_setup.zsh
     ```
   - Se estiver usando `bash`:
     ```bash
     source install/local_setup.bash
     ```

5. Execute o nó:
   ```bash
   ros2 run cg maze
   ```

---

## Segundo passo:
Em outro terminal:
1. Clone o repositório:
   ```bash
   git clone https://github.com/olin-med/Ponderada1
   ```

2. Navegue até o diretório do projeto:
   ```bash
   cd Ponderada1
   ```

3. Compile o projeto:
   ```bash
   colcon build
   ```

4. Fonteie o arquivo de configuração:
   - Se estiver usando `zsh`:
     ```bash
     source install/local_setup.zsh
     ```
   - Se estiver usando `bash`:
     ```bash
     source install/local_setup.bash
     ```

5. Execute o nó:
   ```bash
   ros2 run bolin main
   ```

---

Siga esses passos para cada projeto para garantir a configuração e execução adequadas. Para mais informações ou personalizações, consulte a documentação específica de cada repositório.
