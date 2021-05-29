// template <class T>
// void vector_to_unbounded_array(std::vector<T>& v,
//     boost::numeric::ublas::unbounded_array<T>& arr)
// {
//     for (unsigned ix = 0; ix < v.size(); ix++)
//     {
//         arr[ix] = v[ix];
//     }
// }

// // Checks whether n is a perfect square.
// bool is_perfect_square(int n)
// {
//     int s = sqrt(n);
//     return s*s == n;
// }

// Returns the length of one side of a square matrix.
int get_matrix_dimension(int num_matrix_elements)
{
    return sqrt(num_matrix_elements);
}
