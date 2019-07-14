#pragma once

#include <cstdint>
#include <initializer_list>
#include <vector>

template<uint32_t _N, uint32_t _M>
class Matrix
{
    std::vector<std::vector<double>> data;
public:
    constexpr static uint32_t const I = _M;
    constexpr static uint32_t const J = _N;

    constexpr auto begin() noexcept { return data.begin(); }
    constexpr auto end() noexcept { return data.end(); }

    constexpr Matrix() noexcept
    {
        data.reserve(I);
        for(int i = 0; i < I; i++)
        {
            data.emplace_back(std::vector<double>{});
            data[i].reserve(J);
            for(int j = 0; j < J; j++)
                data[i].emplace_back(0);
        }
    }
    constexpr Matrix(std::initializer_list<std::vector<double>> il) noexcept
    {
        for(auto d : il)
            data.emplace_back(d);
    }
    constexpr Matrix(Matrix const&) = default;
    constexpr Matrix(Matrix&&) noexcept = default;

    constexpr Matrix& operator=(Matrix const&) noexcept = default;
    constexpr Matrix& operator=(Matrix&&) noexcept = default;

    constexpr Matrix operator+(Matrix const& M) const noexcept
    {
        Matrix out;
        for(int i = 0; i < I; i++)
            for(int j = 0; j < J; j++)
                out[i][j] = data[i][j] + M[i][j];
        return out; 
    }

    template<typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
    constexpr Matrix operator*(T const& t) const noexcept
    {
        Matrix out = *this;
        for(auto& i : out)
            for(auto& j : i)
                j *= t;
        return out;
    }

    template<uint32_t _P>
    constexpr Matrix<_N,_P> operator*(Matrix<_M,_P> const& M) const noexcept
    {
        Matrix<_N,_P> out;
        for(int _i = 0; _i < _P; _i++)
            for(int i = 0; i < _M; i++)
                for(int j = 0; j < _N; j++)
                    out[_i][j] += M[_i][i] * data[i][j];

        return out;
    }

    constexpr Matrix operator-(Matrix const& M) const noexcept { return *this + -1*M; }
    constexpr Matrix& operator+=(Matrix const& M) noexcept { return *this = *this + M;}
    constexpr Matrix& operator-=(Matrix const& M) noexcept { return *this = *this - M; }

    constexpr std::vector<double>& operator[](uint32_t idx) noexcept       { return data[idx]; }
    constexpr std::vector<double>  operator[](uint32_t idx) const noexcept { return data[idx]; }


    template<typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
    constexpr Matrix& operator*=(T const& t) noexcept { return *this = *this * t; }

    constexpr auto transpose() const noexcept
    {
        Matrix<I,J> out;
        
        for(int i = 0; i < I; i++)
            for(int j = 0; j < J; j++)
                out[j][i] = data[i][j];

        return out;
    }
};

template<uint32_t M>
constexpr double determinate(Matrix<M,M> mat, double mult = 1)
{
    if constexpr(M == 1) return mult*mat[0][0];
    else
    {
        double result = 0;
        for(auto v : mat)
        {
            Matrix<M-1,M-1> S;
            int idx = 0;
            for(auto vect : mat)
            {
                if(vect == v) continue;
                S[idx++] = [=]{
                    std::vector<double> tv;
                    for(int i = 1; i < vect.size(); i++)
                        tv.emplace_back(vect[i]);
                    return tv;
                }();
            }
            result += mult * determinate(S,v[0]);
            mult = -mult;
        }
        return result;
    }
}

#ifdef _GLIBCXX_IOSTREAM
#include <iomanip>
template<uint32_t N,uint32_t M>
std::ostream& operator<<(std::ostream& out, Matrix<N,M> const& mat) noexcept
{
    for(int i = 0; i < N; i++)
    {
        std::cout << "[";
        for(int j = 0; j < M; j++)
            std::cout << std::setfill(' ') << std::setw(20) << mat[j][i];
        std::cout << "]\n";
    }
    return out;
}
#endif // _GLIBCXX_IOSTREAM


template<uint32_t N,uint32_t M, typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
constexpr auto operator*(T const& t, Matrix<N,M> const& mat) { return mat * t; }

template<int dim>
auto identity = []{
    Matrix<dim,dim> out;
    for(int j = 0; j < dim; j++)
        out[j][j] = 1;
    return out;
}();