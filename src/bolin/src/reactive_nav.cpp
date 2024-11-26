#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <queue>
#include <set>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/get_map.hpp"

struct AStarNode
{
    int row, col;
    double g_cost, h_cost;
    std::shared_ptr<AStarNode> parent;

    AStarNode(int row, int col, double g_cost, double h_cost, std::shared_ptr<AStarNode> parent = nullptr)
        : row(row), col(col), g_cost(g_cost), h_cost(h_cost), parent(parent) {}

    double f_cost() const { return g_cost + h_cost; }

    bool operator<(const AStarNode &other) const
    {
        return f_cost() > other.f_cost(); // Invertido para que o menor f tenha maior prioridade
    }
};

struct NodeComparator
{
    bool operator()(const std::shared_ptr<AStarNode> &a, const std::shared_ptr<AStarNode> &b) const
    {
        return a->f_cost() > b->f_cost();
    }
};

std::vector<std::pair<int, int>> obter_vizinhos(int row, int col)
{
    return {{row + 1, col}, {row - 1, col}, {row, col + 1}, {row, col - 1}};
}

double calcular_heuristica(int row1, int col1, int row2, int col2)
{
    return std::abs(row1 - row2) + std::abs(col1 - col2); // Distância de Manhattan
}

std::vector<std::pair<int, int>> realizar_busca_a_star(const std::vector<std::vector<uint16_t>> &grade, int inicio_row, int inicio_col, int alvo_row, int alvo_col)
{
    int num_rows = grade.size();
    int num_cols = grade[0].size();

    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, NodeComparator> lista_aberta;
    std::set<std::pair<int, int>> lista_fechada;

    lista_aberta.emplace(std::make_shared<AStarNode>(inicio_row, inicio_col, 0, calcular_heuristica(inicio_row, inicio_col, alvo_row, alvo_col)));

    while (!lista_aberta.empty())
    {
        auto atual = lista_aberta.top();
        lista_aberta.pop();

        if (atual->row == alvo_row && atual->col == alvo_col)
        {
            std::vector<std::pair<int, int>> caminho;
            for (auto node = atual; node != nullptr; node = node->parent)
                caminho.emplace_back(node->row, node->col);
            std::reverse(caminho.begin(), caminho.end());
            return caminho;
        }

        lista_fechada.insert({atual->row, atual->col});

        for (const auto &[vizinho_row, vizinho_col] : obter_vizinhos(atual->row, atual->col))
        {
            if (vizinho_row < 0 || vizinho_row >= num_rows || vizinho_col < 0 || vizinho_col >= num_cols)
                continue;
            if (grade[vizinho_row][vizinho_col] == 0 || lista_fechada.count({vizinho_row, vizinho_col}))
                continue;

            double novo_g_cost = atual->g_cost + 1; // Custo uniforme
            double novo_h_cost = calcular_heuristica(vizinho_row, vizinho_col, alvo_row, alvo_col);
            lista_aberta.emplace(std::make_shared<AStarNode>(vizinho_row, vizinho_col, novo_g_cost, novo_h_cost, atual));
        }
    }

    return {}; // Caminho não encontrado
}

class NavegadorLabirinto : public rclcpp::Node
{
public:
    NavegadorLabirinto() : Node("navegador_labirinto")
    {
        cliente_move_cmd_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        cliente_get_map_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");

        RCLCPP_INFO(this->get_logger(), "NavegadorLabirinto inicializado!");

        while (!cliente_move_cmd_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Aguardando o serviço move_command estar disponível...");
        }
        while (!cliente_get_map_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Aguardando o serviço get_map estar disponível...");
        }
    }

    void enviarComandoMovimento(const std::string &direcao)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direcao;
        auto result = cliente_move_cmd_->async_send_request(request,
                                                            [this](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future)
                                                            {
                                                                try
                                                                {
                                                                    auto response = future.get();
                                                                    if (response->success)
                                                                    {
                                                                        movimento_concluido_ = true;
                                                                        RCLCPP_INFO(this->get_logger(), "Movimento executado com sucesso!");
                                                                    }
                                                                    else
                                                                    {
                                                                        RCLCPP_WARN(this->get_logger(), "Falha ao executar o movimento.");
                                                                    }
                                                                }
                                                                catch (const std::exception &e)
                                                                {
                                                                    RCLCPP_ERROR(this->get_logger(), "Falha na chamada do serviço: %s", e.what());
                                                                }
                                                            });
    }

    void solicitarMapa()
    {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto result = cliente_get_map_->async_send_request(request,
                                                           [this](rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future)
                                                           {
                                                               try
                                                               {
                                                                   auto response = future.get();
                                                                   RCLCPP_INFO(this->get_logger(), "Mapa recebido, dimensões: %d x %d", response->occupancy_grid_shape[0], response->occupancy_grid_shape[1]);
                                                                   inicializarMapa(response->occupancy_grid_shape[0], response->occupancy_grid_shape[1], response->occupancy_grid_flattened);
                                                                   mapa_pronto_ = true;
                                                               }
                                                               catch (const std::exception &e)
                                                               {
                                                                   RCLCPP_ERROR(this->get_logger(), "Falha na chamada do serviço: %s", e.what());
                                                               }
                                                           });
    }

    void inicializarMapa(uint8_t altura, uint8_t largura, const std::vector<std::string> &grade_flat)
    {
        dimensoes_grade_ = {altura, largura};
        grade_labirinto_ = std::vector<std::vector<uint16_t>>(altura, std::vector<uint16_t>(largura, 0));
        for (int row = 0; row < altura; row++)
        {
            for (int col = 0; col < largura; col++)
            {
                const auto &celula = grade_flat[row * largura + col];
                if (celula == "b")
                {
                    grade_labirinto_[row][col] = 0;
                }
                else if (celula == "f")
                {
                    grade_labirinto_[row][col] = 1;
                }
                else if (celula == "t")
                {
                    posicao_alvo_ = {row, col};
                    grade_labirinto_[row][col] = 2;
                }
                else if (celula == "r")
                {
                    posicao_inicial_ = {row, col};
                    grade_labirinto_[row][col] = 3;
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Mapa inicializado.");
    }

    void calcularCaminho()
    {
        if (grade_labirinto_.empty())
        {
            return;
        }
        caminho_navegacao_ = realizar_busca_a_star(grade_labirinto_, posicao_inicial_[0], posicao_inicial_[1], posicao_alvo_[0], posicao_alvo_[1]);
        if (!caminho_navegacao_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Caminho calculado com %zu passos.", caminho_navegacao_.size());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Falha ao calcular o caminho.");
        }
    }

    void exibirCaminho()
    {
        if (caminho_navegacao_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Nenhum caminho para exibir.");
            return;
        }

        std::stringstream ss;
        for (const auto &posicao : caminho_navegacao_)
        {
            ss << "(" << posicao.first << ", " << posicao.second << ") ";
        }
        RCLCPP_INFO(this->get_logger(), "Caminho calculado: %s", ss.str().c_str());
    }

    void iniciarNavegacao()
    {
        if (caminho_navegacao_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Nenhum caminho para navegar.");
            return;
        }

        indice_passo_ = 1; // Inicia do primeiro passo após a posição inicial

        // Configura o timer para chamar executarProximoPasso periodicamente
        timer_movimento_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), // Ajuste o período conforme necessário
            std::bind(&NavegadorLabirinto::executarProximoPasso, this));
    }

    void executarProximoPasso()
    {
        if (!movimento_concluido_)
        {
            RCLCPP_INFO(this->get_logger(), "Aguardando conclusão do movimento anterior.");
            return;
        }

        if (indice_passo_ >= caminho_navegacao_.size())
        {
            RCLCPP_INFO(this->get_logger(), "Chegou ao final do caminho.");
            timer_movimento_->cancel(); // Para o timer
            return;
        }

        movimento_concluido_ = false; // Reseta a flag antes de enviar novo comando

        int delta_row = caminho_navegacao_[indice_passo_].first - caminho_navegacao_[indice_passo_ - 1].first;
        int delta_col = caminho_navegacao_[indice_passo_].second - caminho_navegacao_[indice_passo_ - 1].second;

        std::string direcao;
        if (delta_row == 1)
            direcao = "down";
        else if (delta_row == -1)
            direcao = "up";
        else if (delta_col == 1)
            direcao = "right";
        else if (delta_col == -1)
            direcao = "left";

        RCLCPP_INFO(this->get_logger(), "Movendo na direção: %s", direcao.c_str());
        enviarComandoMovimento(direcao);

        indice_passo_++; // Avança para o próximo passo
    }

    // Adiciona um método público para verificar se o mapa está pronto
    bool isMapReady() const
    {
        return mapa_pronto_;
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr cliente_move_cmd_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr cliente_get_map_;
    std::vector<std::vector<uint16_t>> grade_labirinto_;
    std::vector<uint8_t> dimensoes_grade_;
    std::vector<int> posicao_inicial_;
    std::vector<int> posicao_alvo_;
    std::vector<std::pair<int, int>> caminho_navegacao_;
    bool mapa_pronto_ = false;
    bool movimento_concluido_ = true;
    size_t indice_passo_ = 1;
    rclcpp::TimerBase::SharedPtr timer_movimento_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Iniciando o NavegadorLabirinto...");
    auto navegador = std::make_shared<NavegadorLabirinto>();

    navegador->solicitarMapa();

    while (!navegador->isMapReady()) // Usa o método público
    {
        rclcpp::spin_some(navegador); // Processa callbacks e verifica o flag mapa_pronto_
    }

    navegador->calcularCaminho();
    navegador->exibirCaminho();
    navegador->iniciarNavegacao();

    rclcpp::spin(navegador);
    navegador.reset(); // Destrói o node
    rclcpp::shutdown();
}
